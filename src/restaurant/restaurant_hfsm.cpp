/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVrestaurant_hfsmED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCrestaurant_hfsmENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "restaurant_hfsm.h"

namespace bica
{
restaurant_hfsm::restaurant_hfsm() : state_(INIT), myBaseId_("restaurant_hfsm")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

restaurant_hfsm::~restaurant_hfsm()
{
}

void restaurant_hfsm::activateCode()
{
  	deactivateAllDeps();

	state_ = INIT;
	state_ts_ = ros::Time::now();

	Init_activateDeps();
	Init_code_once();

}

bool restaurant_hfsm::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case DELIVERORDER:

	deliverOrder_code_iterative();

	msg.data = "deliverOrder";
	if(deliverOrder_2_idle())
	{

	deactivateAllDeps();

	state_ = IDLE;
	state_ts_ = ros::Time::now();

	idle_activateDeps();
	idle_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FIXORDER:

	fixOrder_code_iterative();

	msg.data = "fixOrder";
	if(fixOrder_2_checkOrder())
	{

	deactivateAllDeps();

	state_ = CHECKORDER;
	state_ts_ = ros::Time::now();

	checkOrder_activateDeps();
	checkOrder_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GRETTINGNEWCUSTOMER:

	grettingNewCustomer_code_iterative();

	msg.data = "grettingNewCustomer";
	if(grettingNewCustomer_2_idle())
	{

	deactivateAllDeps();

	state_ = IDLE;
	state_ts_ = ros::Time::now();

	idle_activateDeps();
	idle_code_once();
	}
	state_pub_.publish(msg);
	break;

	case IDLE:

	idle_code_iterative();

	msg.data = "idle";
	if(idle_2_grettingNewCustomer())
	{

	deactivateAllDeps();

	state_ = GRETTINGNEWCUSTOMER;
	state_ts_ = ros::Time::now();

	grettingNewCustomer_activateDeps();
	grettingNewCustomer_code_once();
	}
	if(idle_2_getOrder())
	{

	deactivateAllDeps();

	state_ = GETORDER;
	state_ts_ = ros::Time::now();

	getOrder_activateDeps();
	getOrder_code_once();
	}
	state_pub_.publish(msg);
	break;

	case SETORDER:

	setOrder_code_iterative();

	msg.data = "setOrder";
	if(setOrder_2_checkOrder())
	{

	deactivateAllDeps();

	state_ = CHECKORDER;
	state_ts_ = ros::Time::now();

	checkOrder_activateDeps();
	checkOrder_code_once();
	}
	state_pub_.publish(msg);
	break;

	case CHECKORDER:

	checkOrder_code_iterative();

	msg.data = "checkOrder";
	if(checkOrder_2_deliverOrder())
	{

	deactivateAllDeps();

	state_ = DELIVERORDER;
	state_ts_ = ros::Time::now();

	deliverOrder_activateDeps();
	deliverOrder_code_once();
	}
	if(checkOrder_2_fixOrder())
	{

	deactivateAllDeps();

	state_ = FIXORDER;
	state_ts_ = ros::Time::now();

	fixOrder_activateDeps();
	fixOrder_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GETORDER:

	getOrder_code_iterative();

	msg.data = "getOrder";
	if(getOrder_2_setOrder())
	{

	deactivateAllDeps();

	state_ = SETORDER;
	state_ts_ = ros::Time::now();

	setOrder_activateDeps();
	setOrder_code_once();
	}
	state_pub_.publish(msg);
	break;

	case INIT:

	Init_code_iterative();

	msg.data = "Init";
	if(Init_2_checkTableStatus())
	{

	deactivateAllDeps();

	state_ = CHECKTABLESTATUS;
	state_ts_ = ros::Time::now();

	checkTableStatus_activateDeps();
	checkTableStatus_code_once();
	}
	state_pub_.publish(msg);
	break;

	case CHECKTABLESTATUS:

	checkTableStatus_code_iterative();

	msg.data = "checkTableStatus";
	if(checkTableStatus_2_idle())
	{

	deactivateAllDeps();

	state_ = IDLE;
	state_ts_ = ros::Time::now();

	idle_activateDeps();
	idle_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
restaurant_hfsm::deactivateAllDeps()
{
};

void
restaurant_hfsm::deliverOrder_activateDeps()
{
}

void
restaurant_hfsm::fixOrder_activateDeps()
{
}

void
restaurant_hfsm::grettingNewCustomer_activateDeps()
{
}

void
restaurant_hfsm::idle_activateDeps()
{
}

void
restaurant_hfsm::setOrder_activateDeps()
{
}

void
restaurant_hfsm::checkOrder_activateDeps()
{
}

void
restaurant_hfsm::getOrder_activateDeps()
{
}

void
restaurant_hfsm::Init_activateDeps()
{
}

void
restaurant_hfsm::checkTableStatus_activateDeps()
{
}



} /* namespace bica */
