/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute( )
{
  const float threshold = 450; //millimeters
  float rot = 0.9;  //rads per second
  const int offset = 5;
  int v;
  static float B=-(M_PI/4*M_PI/4)/log(0.3);
  static float C=1/log(0.5);
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+offset, ldata.end()-offset, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
    
    float angle=(ldata.data()+offset)->angle;
    float dist=(ldata.data()+offset)->dist;
    
    v=0.5*dist;
    if(v>500)  v=500;
    
    rot=exp(-(angle*angle)/B)/(dist/500);
    differentialrobot_proxy->setSpeedBase(v, rot);
    qDebug()<<v<<rot;
    }
        
    
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

/*
   if( (ldata.data()+20)->dist < threshold)
    {
     //   std::cout << ldata.front().dist << std::endl;
      if((ldata.data()+20)->angle>0){
	giro=false;
      }else{
	giro=true;
      }
      
	if(giro){
	differentialrobot_proxy->setSpeedBase(5, Vgiro);//girar derecha
	usleep(rand()%(1500000-100000 + 1) + 100000); 
	
	}
	else{
	  differentialrobot_proxy->setSpeedBase(5, -Vgiro);//girar izquierda
	  usleep(rand()%(1500000-100000 + 1) + 100000); 
	 
	}
    }
    
    else
    {
     
      if(((ldata.data()+20)->dist)>700){
	x=700;
      }else{
	 x=(ldata.data()+20)->dist;
      }
        differentialrobot_proxy->setSpeedBase(x, 0); // velocidad recta
	//giro=not giro;
	
    }   
    */




