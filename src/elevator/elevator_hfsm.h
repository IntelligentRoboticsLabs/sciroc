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
#ifndef ELEVATOR_HFSM_H_
#define ELEVATOR_HFSM_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class elevator_hfsm : public bica::Component
{
public:
  elevator_hfsm();
  virtual ~elevator_hfsm();

  void activateCode();

  	virtual void askForFloor_code_iterative() {};
	virtual void askForFloor_code_once() {};
	virtual void robotAtEnd_code_iterative() {};
	virtual void robotAtEnd_code_once() {};
	virtual void Init_code_iterative() {};
	virtual void Init_code_once() {};
	virtual void robotAtElevator_code_iterative() {};
	virtual void robotAtElevator_code_once() {};
	virtual void waitForDoor_code_iterative() {};
	virtual void waitForDoor_code_once() {};
	virtual void findProxemicPos_code_iterative() {};
	virtual void findProxemicPos_code_once() {};
	virtual void approachElevator_code_iterative() {};
	virtual void approachElevator_code_once() {};
	virtual void advertiseGoal_code_iterative() {};
	virtual void advertiseGoal_code_once() {};
	virtual void getShopList_code_iterative() {};
	virtual void getShopList_code_once() {};

  	virtual bool askForFloor_2_robotAtEnd() {return false;};
	virtual bool waitForDoor_2_askForFloor() {return false;};
	virtual bool askForFloor_2_waitForDoor() {return false;};
	virtual bool Init_2_getShopList() {return false;};
	virtual bool findProxemicPos_2_robotAtElevator() {return false;};
	virtual bool getShopList_2_approachElevator() {return false;};
	virtual bool approachElevator_2_findProxemicPos() {return false;};
	virtual bool robotAtElevator_2_advertiseGoal() {return false;};
	virtual bool advertiseGoal_2_waitForDoor() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  	void deactivateAllDeps();
	void askForFloor_activateDeps();
	void robotAtEnd_activateDeps();
	void Init_activateDeps();
	void robotAtElevator_activateDeps();
	void waitForDoor_activateDeps();
	void findProxemicPos_activateDeps();
	void approachElevator_activateDeps();
	void advertiseGoal_activateDeps();
	void getShopList_activateDeps();


  	static const int ASKFORFLOOR = 0;
	static const int ROBOTATEND = 1;
	static const int INIT = 2;
	static const int ROBOTATELEVATOR = 3;
	static const int WAITFORDOOR = 4;
	static const int FINDPROXEMICPOS = 5;
	static const int APPROACHELEVATOR = 6;
	static const int ADVERTISEGOAL = 7;
	static const int GETSHOPLIST = 8;


  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

} /* namespace bica */

#endif /* ELEVATOR_HFSM_H_ */
