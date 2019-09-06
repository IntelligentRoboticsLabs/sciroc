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

*   THIS SOFTWARE IS PROVelevator_hfsmED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCelevator_hfsmENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "elevator_hfsm.h"

namespace bica
{
elevator_hfsm::elevator_hfsm() : state_(INIT), myBaseId_("elevator_hfsm")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

elevator_hfsm::~elevator_hfsm()
{
}

void elevator_hfsm::activateCode()
{
  	deactivateAllDeps();

	state_ = INIT;
	state_ts_ = ros::Time::now();

	Init_activateDeps();
	Init_code_once();

}

bool elevator_hfsm::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case ASKFORFLOOR:

	askForFloor_code_iterative();

	msg.data = "askForFloor";
	if(askForFloor_2_robotAtEnd())
	{

	deactivateAllDeps();

	state_ = ROBOTATEND;
	state_ts_ = ros::Time::now();

	robotAtEnd_activateDeps();
	robotAtEnd_code_once();
	}
	if(askForFloor_2_waitForDoor())
	{

	deactivateAllDeps();

	state_ = WAITFORDOOR;
	state_ts_ = ros::Time::now();

	waitForDoor_activateDeps();
	waitForDoor_code_once();
	}
	state_pub_.publish(msg);
	break;

	case ROBOTATEND:

	robotAtEnd_code_iterative();

	msg.data = "robotAtEnd";
	state_pub_.publish(msg);
	break;

	case INIT:

	Init_code_iterative();

	msg.data = "Init";
	if(Init_2_getShopList())
	{

	deactivateAllDeps();

	state_ = GETSHOPLIST;
	state_ts_ = ros::Time::now();

	getShopList_activateDeps();
	getShopList_code_once();
	}
	state_pub_.publish(msg);
	break;

	case ROBOTATELEVATOR:

	robotAtElevator_code_iterative();

	msg.data = "robotAtElevator";
	if(robotAtElevator_2_advertiseGoal())
	{

	deactivateAllDeps();

	state_ = ADVERTISEGOAL;
	state_ts_ = ros::Time::now();

	advertiseGoal_activateDeps();
	advertiseGoal_code_once();
	}
	state_pub_.publish(msg);
	break;

	case WAITFORDOOR:

	waitForDoor_code_iterative();

	msg.data = "waitForDoor";
	if(waitForDoor_2_askForFloor())
	{

	deactivateAllDeps();

	state_ = ASKFORFLOOR;
	state_ts_ = ros::Time::now();

	askForFloor_activateDeps();
	askForFloor_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FINDPROXEMICPOS:

	findProxemicPos_code_iterative();

	msg.data = "findProxemicPos";
	if(findProxemicPos_2_robotAtElevator())
	{

	deactivateAllDeps();

	state_ = ROBOTATELEVATOR;
	state_ts_ = ros::Time::now();

	robotAtElevator_activateDeps();
	robotAtElevator_code_once();
	}
	state_pub_.publish(msg);
	break;

	case APPROACHELEVATOR:

	approachElevator_code_iterative();

	msg.data = "approachElevator";
	if(approachElevator_2_findProxemicPos())
	{

	deactivateAllDeps();

	state_ = FINDPROXEMICPOS;
	state_ts_ = ros::Time::now();

	findProxemicPos_activateDeps();
	findProxemicPos_code_once();
	}
	state_pub_.publish(msg);
	break;

	case ADVERTISEGOAL:

	advertiseGoal_code_iterative();

	msg.data = "advertiseGoal";
	if(advertiseGoal_2_waitForDoor())
	{

	deactivateAllDeps();

	state_ = WAITFORDOOR;
	state_ts_ = ros::Time::now();

	waitForDoor_activateDeps();
	waitForDoor_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GETSHOPLIST:

	getShopList_code_iterative();

	msg.data = "getShopList";
	if(getShopList_2_approachElevator())
	{

	deactivateAllDeps();

	state_ = APPROACHELEVATOR;
	state_ts_ = ros::Time::now();

	approachElevator_activateDeps();
	approachElevator_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
elevator_hfsm::deactivateAllDeps()
{
};

void
elevator_hfsm::askForFloor_activateDeps()
{
}

void
elevator_hfsm::robotAtEnd_activateDeps()
{
}

void
elevator_hfsm::Init_activateDeps()
{
}

void
elevator_hfsm::robotAtElevator_activateDeps()
{
}

void
elevator_hfsm::waitForDoor_activateDeps()
{
}

void
elevator_hfsm::findProxemicPos_activateDeps()
{
}

void
elevator_hfsm::approachElevator_activateDeps()
{
}

void
elevator_hfsm::advertiseGoal_activateDeps()
{
}

void
elevator_hfsm::getShopList_activateDeps()
{
}



} /* namespace bica */
