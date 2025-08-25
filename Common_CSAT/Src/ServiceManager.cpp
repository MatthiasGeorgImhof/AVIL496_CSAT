/*
 * ServiceManager.cpp
 *
 *  Created on: Dec 31, 2024
 *      Author: mgi
 */

#include "ServiceManager.hpp"
#include <algorithm>
#include "Allocator.hpp"

void ServiceManager::initializeServices(uint32_t now) const
{
	for(auto handler : handlers_)
	{
		handler.task->initialize(now);
	}
}

void ServiceManager::handleMessage(std::shared_ptr<CyphalTransfer> transfer) const
{
	for(auto handler : handlers_)
	{
		if (handler.port_id == transfer->metadata.port_id)
		{
			handler.task->handleMessage(transfer);
		}
	}
}

void ServiceManager::handleServices() const
{
	for(auto handler : handlers_)
	{
		handler.task->handleTask();
	}
}
