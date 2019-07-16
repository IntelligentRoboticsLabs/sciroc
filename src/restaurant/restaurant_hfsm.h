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
#ifndef RESTAURANT_HFSM_H_
#define RESTAURANT_HFSM_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class restaurant_hfsm : public bica::Component
{
public:
  restaurant_hfsm();
  virtual ~restaurant_hfsm();

  void activateCode();

  virtual void deliverOrder_code_iterative() {};
	virtual void deliverOrder_code_once() {};
	virtual void fixOrder_code_iterative() {};
	virtual void fixOrder_code_once() {};
	virtual void grettingNewCustomer_code_iterative() {};
	virtual void grettingNewCustomer_code_once() {};
	virtual void idle_code_iterative() {};
	virtual void idle_code_once() {};
	virtual void setOrder_code_iterative() {};
	virtual void setOrder_code_once() {};
	virtual void checkOrder_code_iterative() {};
	virtual void checkOrder_code_once() {};
	virtual void getOrder_code_iterative() {};
	virtual void getOrder_code_once() {};
	virtual void Init_code_iterative() {};
	virtual void Init_code_once() {};
	virtual void checkTableStatus_code_iterative() {};
	virtual void checkTableStatus_code_once() {};

  virtual bool fixOrder_2_checkOrder() {return false;};
	virtual bool idle_2_grettingNewCustomer() {return false;};
	virtual bool checkOrder_2_deliverOrder() {return false;};
	virtual bool idle_2_getOrder() {return false;};
	virtual bool checkTableStatus_2_idle() {return false;};
	virtual bool checkOrder_2_fixOrder() {return false;};
	virtual bool deliverOrder_2_idle() {return false;};
	virtual bool getOrder_2_setOrder() {return false;};
	virtual bool Init_2_checkTableStatus() {return false;};
	virtual bool setOrder_2_checkOrder() {return false;};
	virtual bool grettingNewCustomer_2_idle() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  void deactivateAllDeps();
	void deliverOrder_activateDeps();
	void fixOrder_activateDeps();
	void grettingNewCustomer_activateDeps();
	void idle_activateDeps();
	void setOrder_activateDeps();
	void checkOrder_activateDeps();
	void getOrder_activateDeps();
	void Init_activateDeps();
	void checkTableStatus_activateDeps();


  static const int DELIVERORDER = 0;
	static const int FIXORDER = 1;
	static const int GRETTINGNEWCUSTOMER = 2;
	static const int IDLE = 3;
	static const int SETORDER = 4;
	static const int CHECKORDER = 5;
	static const int GETORDER = 6;
	static const int INIT = 7;
	static const int CHECKTABLESTATUS = 8;


  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

} /* namespace bica */

#endif /* RESTAURANT_HFSM_H_ */
