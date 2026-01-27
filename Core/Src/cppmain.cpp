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
#include "HeapAllocation.hpp"
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
#include "OrientationTracker.hpp"
#include "OrientationService.hpp"
#include "TaskOrientationService.hpp"
#include "PositionTracker9D.hpp"
#include "TaskPositionService.hpp"
#include "GNSS.hpp"


#include "au.hh"
#include "au.hpp"

#include "Logger.hpp"

constexpr size_t O1HEAP_SIZE = 65536;
using LocalHeap = HeapAllocation<O1HEAP_SIZE>;

CanardAdapter canard_adapter;
LoopardAdapter loopard_adapter;

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

constexpr uint16_t endian_swap(uint16_t num) {return (num>>8) | (num<<8); };
constexpr int16_t endian_swap(int16_t num) {return (num>>8) | (num<<8); };

template<typename T, typename... Args>
static void register_task_with_heap(RegistrationManager& rm, Args&&... args)
{
    static SafeAllocator<T, LocalHeap> alloc;
    rm.add(alloc_unique_custom<T>(alloc, std::forward<Args>(args)...));
}

void cppmain()
{
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

	LocalHeap::initialize();
	O1HeapInstance *o1heap = LocalHeap::getO1Heap();

	using LoopardCyphal = Cyphal<LoopardAdapter>;
	loopard_adapter.memory_allocate = LocalHeap::loopardMemoryAllocate;
	loopard_adapter.memory_free = LocalHeap::loopardMemoryDeallocate;
	LoopardCyphal loopard_cyphal(&loopard_adapter);
	loopard_cyphal.setNodeID(cyphal_node_id);

	using CanardCyphal = Cyphal<CanardAdapter>;
	canard_adapter.ins = canardInit(LocalHeap::canardMemoryAllocate, LocalHeap::canardMemoryDeallocate);
	canard_adapter.que = canardTxInit(512, CANARD_MTU_CAN_CLASSIC);
	CanardCyphal canard_cyphal(&canard_adapter);
	canard_cyphal.setNodeID(cyphal_node_id);

	//	Logger::setCyphalCanardAdapter(&canard_cyphal);
	std::tuple<CanardCyphal> canard_adapters = { canard_cyphal };
	std::tuple<> empty_adapters = {} ;

	RegistrationManager registration_manager;
	SubscriptionManager subscription_manager;

//	constexpr auto ACS_LOGIC_3V3 = CIRCUITS::CIRCUIT_0;
//	constexpr auto MRAM_3V3 = CIRCUITS::CIRCUIT_1;
//	constexpr auto GYRO_3V3 = CIRCUITS::CIRCUIT_2;
//	constexpr auto CLK_3V3 = CIRCUITS::CIRCUIT_3;
//	constexpr auto TEMP_3V3 = CIRCUITS::CIRCUIT_4;
//	constexpr auto MAG_3V3 = CIRCUITS::CIRCUIT_5;
//	constexpr auto SUN_3V3 = CIRCUITS::CIRCUIT_6;
//	constexpr auto GPS_3V3 = CIRCUITS::CIRCUIT_7;

//	constexpr uint8_t GPIO_EXPANDER = 32;
//	using PowerSwitchConfig = I2C_Register_Config<hi2c4, GPIO_EXPANDER>;
//	using PowerSwitchTransport = I2CRegisterTransport<PowerSwitchConfig>;
//	PowerSwitchTransport ps_transport;
//	PowerSwitch<PowerSwitchTransport> power_switch(ps_transport, GPIOC, GPIO_POWER_RST_Pin);
//    power_switch.releaseReset();

	constexpr uint8_t uuid[] = {0x2b, 0x8c, 0xda, 0x5f, 0x91, 0x3e, 0x47, 0xa2, 0xb5, 0x07, 0x8f, 0xd3, 0x64, 0xe9, 0x1c, 0x70};
	constexpr char node_name[50] = "AVIL496_CSAT";

	using TSHeart = TaskSendHeartBeat<CanardCyphal>;
	register_task_with_heap<TSHeart>(registration_manager, 2000, 100, 0, canard_adapters);

	using TPHeart = TaskProcessHeartBeat<CanardCyphal>;
	register_task_with_heap<TPHeart>(registration_manager, 2000, 100, canard_adapters);

	using TSendNodeList = TaskSendNodePortList<CanardCyphal>;
	register_task_with_heap<TSendNodeList>(registration_manager, &registration_manager, 10000, 100, 0, canard_adapters);

	using TSubscribeNodeList = TaskSubscribeNodePortList<CanardCyphal>;
	register_task_with_heap<TSubscribeNodeList>(registration_manager, &subscription_manager, 10000, 100, canard_adapters);

	using TRespondInfo = TaskRespondGetInfo<CanardCyphal>;
	register_task_with_heap<TRespondInfo>(registration_manager, uuid, node_name, 10000, 100, canard_adapters);

	using TRequestInfo = TaskRequestGetInfo<CanardCyphal>;
	register_task_with_heap<TRequestInfo>(registration_manager, 10000, 100, 11, 0, canard_adapters);

	using TBlink = TaskBlinkLED;
	register_task_with_heap<TBlink>(registration_manager, GPIOB, LED1_Pin, 1000, 100);

	using TCheckMem = TaskCheckMemory;
	register_task_with_heap<TCheckMem>(registration_manager, o1heap, 2000, 100);

//    power_switch.releaseReset();
//    power_switch.on(GYRO_3V3);
//    power_switch.on(MAG_3V3);
//    power_switch.on(GPS_3V3);
//
//	constexpr uint8_t INA226 = 64;
//	using PowerMonitorConfig = I2C_Register_Config<hi2c2, INA226>;
//	using PowerMonitorTransport = I2CRegisterTransport<PowerMonitorConfig>;
//	PowerMonitorTransport pm_transport;
//	PowerMonitor<PowerMonitorTransport> power_monitor(pm_transport);
//
//	constexpr MagnetometerCalibration aux_mmc_calibration = {
//	    { 3.19048657e+03f,  3.21802648e+03f, -1.86857329e+01f, },
//	    {{
//	        { 1.0f,	0.0f, 0.0f },
//	        { 0.0f, 1.0f, 0.0f },
//	        { 0.0f, 0.0f, 1.0f }
//	    }}
//	};
//	using IMUConfigType = SPI_Register_Config<hspi2, GPIO_SPI2_GYRO_CS_Pin, 128>;
//	IMUConfigType imu_config(GPIOD);
//	SPIRegisterTransport<IMUConfigType> imu_transport(imu_config);
//	BMI270_MMC5983<SPIRegisterTransport<IMUConfigType>> imu(imu_transport, aux_mmc_calibration);
//	(void) imu.readChipID();
//	(void) imu.readChipID();
//	HAL_Delay(5000);
//	assert(imu.initialize());
//	assert(imu.configure());
//
//
//	constexpr MagnetometerCalibration spi_mmc_calibration = {
//    { -4.78483917e+02f,  2.32297897e+03f,  6.76868257e+02f, },
//    {{
//	        { 1.0f,	0.0f, 0.0f },
//	        { 0.0f, 1.0f, 0.0f },
//	        { 0.0f, 0.0f, 1.0f }
//    }}
//	};
//	using MagConfigType = SPI_Register_Config<hspi1, GPIO_SPI1_MAG_CS_Pin, 128>;
//	MagConfigType mag_config(GPIOE);
//	SPIRegisterTransport<MagConfigType> mag_transport(mag_config);
//    MMC5983<SPIRegisterTransport<MagConfigType>> mag(mag_transport, spi_mmc_calibration);
//	assert(mag.initialize());
//
////	using MramConfig = SPI_Config<hspi3, &GPIOG_object, GPIO_SPI3_MRAM_CS_Pin, 128>;
////    using MramTransport = SPITransport<MramConfig>;
////    MramTransport mram_transport;
////    MR25H10<MramTransport> mram(mram_transport);
////    (void) mag.readChipID();
//
//	using OrientationTrackerType = AccGyrMagOrientationTracker<7,6>;
//	using IMUType = BMI270_MMC5983<SPIRegisterTransport<IMUConfigType>>;
//	using OrientationType = AccGyrMagOrientation<OrientationTrackerType, IMUType, IMUType>;
//	using OrientationTask = TaskOrientationService<OrientationType, Cyphal<CanardAdapter>>;
//	OrientationTrackerType orientation_tracker;
//	OrientationType orientation(&hrtc, orientation_tracker, imu, imu);
//	using OrientationTask = TaskOrientationService<OrientationType, Cyphal<CanardAdapter>>;
//	register_task_with_heap<OrientationTask>(registration_manager, orientation, 100, 5, 0, canard_adapters);
//
//
//	using PositionTrackerType = PositionTracker9D;
//	using PositionType = GNSSandAccelPosition<PositionTrackerType, SimulatedGNSS, IMUType, OrientationType, SubtractGravityInNED>;
//	using PositionTask = TaskPositionService<PositionType, Cyphal<CanardAdapter>>;
//	SimulatedGNSS gnss(25);
//	PositionTrackerType position_tracker;
//	PositionType Position(&hrtc, position_tracker, gnss, imu, orientation, 100, 1);
//	using PositionTask = TaskPositionService<PositionType, Cyphal<CanardAdapter>>;
//	register_task_with_heap<PositionTask>(registration_manager, Position, 100, 5, 0, canard_adapters);

	subscription_manager.subscribe<SubscriptionManager::MessageTag>(registration_manager.getSubscriptions(), canard_adapters);
    subscription_manager.subscribe<SubscriptionManager::ResponseTag>(registration_manager.getServers(), canard_adapters);
    subscription_manager.subscribe<SubscriptionManager::RequestTag>(registration_manager.getClients(), canard_adapters);

    ServiceManager service_manager(registration_manager.getHandlers());
	service_manager.initializeServices(HAL_GetTick());

	static SafeAllocator<CyphalTransfer, LocalHeap> allocator;
	LoopManager loop_manager(allocator);

	while(1)
	{
		loop_manager.CanProcessTxQueue(&canard_adapter, &hcan1);
		loop_manager.CanProcessRxQueue(&canard_cyphal, &service_manager, empty_adapters, can_rx_buffer);
		loop_manager.LoopProcessRxQueue(&loopard_cyphal, &service_manager, empty_adapters);
		service_manager.handleServices();

//		char buffer[256];
////		sprintf(buffer, "SPI: %d %d %d \r\n", HAL_SPI_GetState(&hspi1), HAL_SPI_GetState(&hspi2), HAL_SPI_GetState(&hspi3));
//
////		static constexpr uint8_t BMI270_READ_BIT = 0x80;
////		HAL_GPIO_WritePin(GPIO_SPI2_GYRO_CS_GPIO_Port, GPIO_SPI2_GYRO_CS_Pin, GPIO_PIN_RESET);
////		uint8_t tx[1] { 0x00 | BMI270_READ_BIT };
////		uint8_t rx[2] {};
////		bool okt = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), 100) == HAL_OK;
////		bool okr = HAL_SPI_TransmitReceive(&hspi2, rx, rx, sizeof(rx), 100) == HAL_OK;
////		HAL_GPIO_WritePin(GPIO_SPI2_GYRO_CS_GPIO_Port, GPIO_SPI2_GYRO_CS_Pin, GPIO_PIN_SET);
////		sprintf(buffer, "SPI direct: %d %x\r\n", HAL_SPI_GetState(&hspi2), rx[1]);
////		CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
////		(void) okr;
////		(void) okt;
//
////		HAL_Delay(250);
//		auto imu_acc = imu.readAccelerometer();
//		auto imu_gyr = imu.readGyroscope();
//		auto imu_tmp = imu.readThermometer();
//		auto imu_mag = imu.readMagnetometer();

////		auto mag_chip = mag.readChipID();
////		auto mag_tmp = mag.readRawThermometer();
//		auto mag_mag = mag.readMagnetometer();
//
//		sprintf(buffer, "SPI IMU: (%+f %+f %+f) (%+f %+f %+f) (%f) (%+e %+e %+e) (%+e %+e %+e)\r\n",
//				imu_acc.value()[0].in(au::metersPerSecondSquaredInBodyFrame),
//				imu_acc.value()[1].in(au::metersPerSecondSquaredInBodyFrame),
//				imu_acc.value()[2].in(au::metersPerSecondSquaredInBodyFrame),
//				imu_gyr.value()[0].in(au::degreesPerSecondInBodyFrame),
//				imu_gyr.value()[1].in(au::degreesPerSecondInBodyFrame),
//				imu_gyr.value()[2].in(au::degreesPerSecondInBodyFrame),
//				imu_tmp.value().in(au::celsius_qty),
//				imu_mag.value()[0].in(au::nano(au::teslaInBodyFrame)), imu_mag.value()[1].in(au::nano(au::teslaInBodyFrame)), imu_mag.value()[2].in(au::nano(au::teslaInBodyFrame)),
//				mag_mag.value()[0].in(au::nano(au::teslaInBodyFrame)), mag_mag.value()[1].in(au::nano(au::teslaInBodyFrame)), mag_mag.value()[2].in(au::nano(au::teslaInBodyFrame)));
//
////		HAL_Delay(250);
//		auto aux_mag = imu.readRawMagnetometer();
//		auto spi_mag = mag.readRawMagnetometer();
//		(void) aux_mag;
//		(void) spi_mag;
//		sprintf(buffer, "MAGs %ld %ld %ld %ld %ld %ld \r\n",
//				aux_mag[0], aux_mag[1], aux_mag[2], spi_mag[0], spi_mag[1], spi_mag[2]);
//
//
////	    auto imu_id = imu.readChipID();
////	    auto mag_id = mag.readChipID();
////	    auto mram_id = mram.readStatus();
////	    auto power = power_switch.getState();
////
////		PowerMonitorData data;
////		power_monitor(data);
////		sprintf(buffer, "IMU ID: %u %u %u %u\r\nINA226: %4x %4x % 6d % 6d % 6d % 6d\r\n", power, imu_id.value(), mag_id.value(), mram_id.value(),
////			  data.manufacturer_id, data.die_id, data.voltage_shunt_uV, data.voltage_bus_mV, data.power_mW, data.current_uA);
//
//		CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
//		HAL_Delay(100);
	}
}
