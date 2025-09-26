#include <cppmain.h>
#include <cpphal.h>
#include "usb_device.h"

#include <memory>

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "usbd_cdc_if.h"

#include "o1heap.h"
#include "canard.h"
#include "serard.h"

#include "cyphal.hpp"
#include "canard_adapter.hpp"
#include "serard_adapter.hpp"
#include "loopard_adapter.hpp"

#ifndef NUNAVUT_ASSERT
#define NUNAVUT_ASSERT(x) assert(x)
#endif
#include "uavcan/diagnostic/Record_1_1.h"

#include <CircularBuffer.hpp>
#include <ArrayList.hpp>
#include "Allocator.hpp"
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp"
#include "SubscriptionManager.hpp"
#include "ProcessRxQueue.hpp"
#include "TaskCheckMemory.hpp"
#include "TaskBlinkLED.hpp"
#include "TaskSendHeartBeat.hpp"
#include "TaskProcessHeartBeat.hpp"
#include "TaskSendNodePortList.hpp"
#include "TaskSubscribeNodePortList.hpp"
#include "TaskRespondGetInfo.hpp"
#include "TaskRequestGetInfo.hpp"
#include "PowerSwitch.hpp"
#include "PowerMonitor.hpp"
#include "BMI270.hpp"
#include "BMI270_MMC5983.hpp"
#include "MMC5983.hpp"
#include "MR25H10.hpp"
#include "MMC5983.hpp"

#include "au.hh"
#include "au.hpp"

#include "Logger.hpp"

constexpr size_t O1HEAP_SIZE = 65536;
uint8_t o1heap_buffer[O1HEAP_SIZE] __attribute__ ((aligned (O1HEAP_ALIGNMENT)));
O1HeapInstance *o1heap;

void* canardMemoryAllocate(CanardInstance *const /*canard*/, const size_t size)
{
	return o1heapAllocate(o1heap, size);
}

void canardMemoryDeallocate(CanardInstance *const /*canard*/, void *const pointer)
{
	o1heapFree(o1heap, pointer);
}

void* serardMemoryAllocate(void *const /*user_reference*/, const size_t size)
{
	return o1heapAllocate(o1heap, size);
}

void serardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer)
{
	o1heapFree(o1heap, pointer);
};

#ifndef CYPHAL_NODE_ID
#define CYPHAL_NODE_ID 31
#endif

constexpr CyphalNodeID cyphal_node_id = CYPHAL_NODE_ID;

constexpr size_t CAN_RX_BUFFER_SIZE = 64;
CircularBuffer<CanRxFrame, CAN_RX_BUFFER_SIZE> can_rx_buffer;

enum class AUX_POWER_SWITCH : uint8_t
{
	ACS_LOGIC_3V3 = 0,
	MRAM_3V3 = 1,
	GYRO_3V3 = 2,
	CLK_3V3 = 3,
	TEMP_3V3 = 4,
	MAG_3V3 = 5,
	SUN_3V3 = 6,
	GPS_3V3 = 7,
};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t num_messages = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
	log(LOG_LEVEL_TRACE, "HAL_CAN_RxFifo0MsgPendingCallback %d\r\n", num_messages);
	for(uint32_t n=0; n<num_messages; ++n)
	{
		if (can_rx_buffer.is_full()) return;

		CanRxFrame &frame = can_rx_buffer.next();
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &frame.header, frame.data);
	}
}

uint16_t endian_swap(uint16_t num) {return (num>>8) | (num<<8); };
int16_t endian_swap(int16_t num) {return (num>>8) | (num<<8); };

void cppmain()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_POWER_RST_Pin, GPIO_PIN_SET);
	constexpr uint8_t GPIO_EXPANDER = 32;
	using SwitchConfig = I2C_Config<hi2c2, GPIO_EXPANDER>;
    using SwitchTransport = I2CTransport<SwitchConfig>;
    SwitchTransport switch_transport;
    PowerSwitch<SwitchTransport> power_switch(switch_transport);
    power_switch.setState(0x00);

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

	CAN_FilterTypeDef filter = {};
	filter.FilterIdHigh = 0x1fff;
	filter.FilterIdLow = 0xffff;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_16BIT;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1, &filter);

	o1heap = o1heapInit(o1heap_buffer, O1HEAP_SIZE);
	O1HeapAllocator<CanardRxTransfer> alloc(o1heap);

	LoopardAdapter loopard_adapter;
	Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);
	loopard_cyphal.setNodeID(cyphal_node_id);

	CanardAdapter canard_adapter;
	canard_adapter.ins = canardInit(&canardMemoryAllocate, &canardMemoryDeallocate);
	canard_adapter.que = canardTxInit(512, CANARD_MTU_CAN_CLASSIC);
	Cyphal<CanardAdapter> canard_cyphal(&canard_adapter);
	canard_cyphal.setNodeID(cyphal_node_id);

//	Logger::setCyphalCanardAdapter(&canard_cyphal);
	std::tuple<Cyphal<CanardAdapter>> canard_adapters = { canard_cyphal };
	std::tuple<> empty_adapters = {} ;

	RegistrationManager registration_manager;
	SubscriptionManager subscription_manager;
	registration_manager.subscribe(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
	registration_manager.subscribe(uavcan_node_port_List_1_0_FIXED_PORT_ID_);
	registration_manager.subscribe(uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_);
	registration_manager.publish(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
	registration_manager.publish(uavcan_node_port_List_1_0_FIXED_PORT_ID_);
	registration_manager.publish(uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_);

	O1HeapAllocator<TaskSendHeartBeat<Cyphal<CanardAdapter>>> alloc_TaskSendHeartBeat(o1heap);
	registration_manager.add(allocate_unique_custom<TaskSendHeartBeat<Cyphal<CanardAdapter>>>(alloc_TaskSendHeartBeat, 2000, 100, 0, canard_adapters));

	O1HeapAllocator<TaskProcessHeartBeat<Cyphal<CanardAdapter>>> alloc_TaskProcessHeartBeat(o1heap);
	registration_manager.add(allocate_unique_custom<TaskProcessHeartBeat<Cyphal<CanardAdapter>>>(alloc_TaskProcessHeartBeat, 2000, 100, canard_adapters));

	O1HeapAllocator<TaskSendNodePortList<Cyphal<CanardAdapter>>> alloc_TaskSendNodePortList(o1heap);
	registration_manager.add(allocate_unique_custom<TaskSendNodePortList<Cyphal<CanardAdapter>>>(alloc_TaskSendNodePortList, &registration_manager, 10000, 100, 0, canard_adapters));

	O1HeapAllocator<TaskSubscribeNodePortList<Cyphal<CanardAdapter>>> alloc_TaskSubscribeNodePortList(o1heap);
	registration_manager.add(allocate_unique_custom<TaskSubscribeNodePortList<Cyphal<CanardAdapter>>>(alloc_TaskSubscribeNodePortList, &subscription_manager, 10000, 100, canard_adapters));

	constexpr uint8_t uuid[] = {0x2b, 0x8c, 0xda, 0x5f, 0x91, 0x3e, 0x47, 0xa2, 0xb5, 0x07, 0x8f, 0xd3, 0x64, 0xe9, 0x1c, 0x70};
	constexpr char node_name[50] = "AVIL496_CSAT";
	O1HeapAllocator<TaskRespondGetInfo<Cyphal<CanardAdapter>>> alloc_TaskRespondGetInfo(o1heap);
	registration_manager.add(allocate_unique_custom<TaskRespondGetInfo<Cyphal<CanardAdapter>>>(alloc_TaskRespondGetInfo, uuid, node_name, 10000, 100, canard_adapters));

	O1HeapAllocator<TaskRequestGetInfo<Cyphal<CanardAdapter>>> alloc_TaskRequestGetInfo(o1heap);
	registration_manager.add(allocate_unique_custom<TaskRequestGetInfo<Cyphal<CanardAdapter>>>(alloc_TaskRequestGetInfo, 10000, 100, 13, 0, canard_adapters));

	O1HeapAllocator<TaskBlinkLED> alloc_TaskBlinkLED(o1heap);
	registration_manager.add(allocate_unique_custom<TaskBlinkLED>(alloc_TaskBlinkLED, LED1_GPIO_Port, LED1_Pin, 1000, 100));

	O1HeapAllocator<TaskCheckMemory> alloc_TaskCheckMemory(o1heap);
	registration_manager.add(allocate_unique_custom<TaskCheckMemory>(alloc_TaskCheckMemory, o1heap, 2000, 100));

    subscription_manager.subscribe<SubscriptionManager::MessageTag>(registration_manager.getSubscriptions(), canard_adapters);
    subscription_manager.subscribe<SubscriptionManager::ResponseTag>(registration_manager.getServers(), canard_adapters);
    subscription_manager.subscribe<SubscriptionManager::RequestTag>(registration_manager.getClients(), canard_adapters);

    ServiceManager service_manager(registration_manager.getHandlers());
	service_manager.initializeServices(HAL_GetTick());

    power_switch.setState(0xff);

	constexpr uint8_t INA226 = 64;
	using MonitorConfig = I2C_Config<hi2c2, INA226>;
    using MonitorTransport = I2CTransport<MonitorConfig>;
    MonitorTransport monitor_transport;
	PowerMonitor<MonitorTransport> power_monitor(monitor_transport);

	constexpr MagnetometerCalibration aux_mmc_calibration = {
	    { au::make_quantity<au::TeslaInBodyFrame>(2.05216351e-05f), au::make_quantity<au::TeslaInBodyFrame>(1.88477490e-05f), au::make_quantity<au::TeslaInBodyFrame>(2.57038804e-07f) },
	    {{
	        { 0.89675333f,  0.0026198f,  -0.00259815f },
	        { 0.0026198f,   0.91909188f,  0.003695f    },
	        { -0.00259815f, 0.003695f,    0.87256288f  }
	    }}
	};
	using IMUConfigType = SPI_Config<hspi2, GPIO_SPI2_GYRO_CS_Pin, 128>;
	IMUConfigType imu_config(GPIOD);
	SPITransport<IMUConfigType> imu_transport(imu_config);
	BMI270_MMC5983<SPITransport<IMUConfigType>> imu(imu_transport, aux_mmc_calibration);
	(void) imu.readChipID();
	(void) imu.readChipID();
	HAL_Delay(5000);
	assert(imu.initialize());
	assert(imu.configure());


	constexpr MagnetometerCalibration spi_mmc_calibration = {
		{ au::make_quantity<au::TeslaInBodyFrame>(-1.28120647e-06f), au::make_quantity<au::TeslaInBodyFrame>(1.11679164e-05f), au::make_quantity<au::TeslaInBodyFrame>(5.11724020e-06f) },
		{{
				{ 0.99498789f, 0.01241906f, 0.00664294f, },
				 { 0.01241906f, 1.00374665f, 0.00229451f,},
				 { 0.00664294f, 0.00229451f, 0.95963044f}
		}}
	};
	using MagConfigType = SPI_Config<hspi1, GPIO_SPI1_MAG_CS_Pin, 128>;
	MagConfigType mag_config(GPIOE);
	SPITransport<MagConfigType> mag_transport(mag_config);
    MMC5983<SPITransport<MagConfigType>> mag(mag_transport, spi_mmc_calibration);
	assert(mag.initialize());

//	using MramConfig = SPI_Config<hspi3, &GPIOG_object, GPIO_SPI3_MRAM_CS_Pin, 128>;
//    using MramTransport = SPITransport<MramConfig>;
//    MramTransport mram_transport;
//    MR25H10<MramTransport> mram(mram_transport);
//    (void) mag.readChipID();

    O1HeapAllocator<CyphalTransfer> allocator(o1heap);
	LoopManager loop_manager(allocator);
	while(1)
	{
		char buffer[256];
		loop_manager.CanProcessTxQueue(&canard_adapter, &hcan1);
		loop_manager.CanProcessRxQueue(&canard_cyphal, &service_manager, empty_adapters, can_rx_buffer);
		loop_manager.LoopProcessRxQueue(&loopard_cyphal, &service_manager, empty_adapters);
		service_manager.handleServices();

//		sprintf(buffer, "SPI: %d %d %d \r\n", HAL_SPI_GetState(&hspi1), HAL_SPI_GetState(&hspi2), HAL_SPI_GetState(&hspi3));

//		static constexpr uint8_t BMI270_READ_BIT = 0x80;
//		HAL_GPIO_WritePin(GPIO_SPI2_GYRO_CS_GPIO_Port, GPIO_SPI2_GYRO_CS_Pin, GPIO_PIN_RESET);
//		uint8_t tx[1] { 0x00 | BMI270_READ_BIT };
//		uint8_t rx[2] {};
//		bool okt = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), 100) == HAL_OK;
//		bool okr = HAL_SPI_TransmitReceive(&hspi2, rx, rx, sizeof(rx), 100) == HAL_OK;
//		HAL_GPIO_WritePin(GPIO_SPI2_GYRO_CS_GPIO_Port, GPIO_SPI2_GYRO_CS_Pin, GPIO_PIN_SET);
//		sprintf(buffer, "SPI direct: %d %x\r\n", HAL_SPI_GetState(&hspi2), rx[1]);
//		CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
//		(void) okr;
//		(void) okt;

//		HAL_Delay(250);
		auto imu_acc = imu.readAccelerometer();
		auto imu_gyr = imu.readGyroscope();
		auto imu_tmp = imu.readThermometer();
		auto imu_mag = imu.readMagnetometer();

//		auto mag_chip = mag.readChipID();
//		auto mag_tmp = mag.readRawThermometer();
		auto mag_mag = mag.readMagnetometer();

		sprintf(buffer, "SPI IMU: (%f %f %f) (%f %f %f) (%f) (%e %e %e) (%e %e %e)\r\n",
				imu_acc.value()[0].in(au::metersPerSecondSquaredInBodyFrame),
				imu_acc.value()[1].in(au::metersPerSecondSquaredInBodyFrame),
				imu_acc.value()[2].in(au::metersPerSecondSquaredInBodyFrame),
				imu_gyr.value()[0].in(au::degreesPerSecondInBodyFrame),
				imu_gyr.value()[1].in(au::degreesPerSecondInBodyFrame),
				imu_gyr.value()[2].in(au::degreesPerSecondInBodyFrame),
				imu_tmp.value().in(au::celsius_qty),
				imu_mag.value()[0].in(au::nano(au::teslaInBodyFrame)), imu_mag.value()[1].in(au::nano(au::teslaInBodyFrame)), imu_mag.value()[2].in(au::nano(au::teslaInBodyFrame)),
				mag_mag.value()[0].in(au::nano(au::teslaInBodyFrame)), mag_mag.value()[1].in(au::nano(au::teslaInBodyFrame)), mag_mag.value()[2].in(au::nano(au::teslaInBodyFrame)));

//		HAL_Delay(250);
//		auto mag_chip = mag.readChipID();
//		auto mag_tmp = mag.readRawThermometer();
//		auto mag_mag = mag.readRawMagnetometer();
//		sprintf(buffer, "SPI MAG %d: (%d) (%ld %ld %ld)\r\n",
//				mag_chip.value(), mag_tmp, mag_mag[0], mag_mag[1], mag_mag[2]);


//	    auto imu_id = imu.readChipID();
//	    auto mag_id = mag.readChipID();
//	    auto mram_id = mram.readStatus();
//	    auto power = power_switch.getState();
//
//		PowerMonitorData data;
//		power_monitor(data);
//		sprintf(buffer, "IMU ID: %u %u %u %u\r\nINA226: %4x %4x % 6d % 6d % 6d % 6d\r\n", power, imu_id.value(), mag_id.value(), mram_id.value(),
//			  data.manufacturer_id, data.die_id, data.voltage_shunt_uV, data.voltage_bus_mV, data.power_mW, data.current_uA);

		CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
		HAL_Delay(100);
	}
}
