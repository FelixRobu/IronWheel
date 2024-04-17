/****************************************************************************
 *
 *   Copyright 2019 NXP.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_race.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

uint8_t get_num_vectors(Vector &vec1, Vector &vec2) {
	uint8_t numVectors = 0;
	if(!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) numVectors++;
	if(!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) numVectors++;
	return numVectors;
}

Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num) {
	Vector vec;
	if(num == 1) {
		vec.m_x0 = pixy.m0_x0;
		vec.m_x1 = pixy.m0_x1;
		vec.m_y0 = pixy.m0_y0;
		vec.m_y1 = pixy.m0_y1;
	}
	if(num == 2) {
		vec.m_x0 = pixy.m1_x0;
		vec.m_x1 = pixy.m1_x1;
		vec.m_y0 = pixy.m1_y0;
		vec.m_y1 = pixy.m1_y1;
	}
	float alpha = 0.017;
	int x = 79 / 2;
	int x0 = vec.m_x0;
	int x1 = vec.m_x1;
	x0 -= x;
	x1 -= x;
	x0 /= (1.0f + alpha * vec.m_y0);
	x1 /= (1.0f + alpha * vec.m_y1);
	x0 += x;
	x1 += x;
	vec.m_x0 = x0;
	vec.m_x1 = x1;
	return vec;
}


roverControl raceTrack(const pixy_vector_s &pixy)
{
	//Vector main_vec;
	Vector vec1 = copy_vectors(pixy, 1);
	Vector vec2 = copy_vectors(pixy, 2);
	uint8_t frameWidth = 79;
	//uint8_t frameHeight = 52;
	int16_t window_center = (frameWidth / 2);
	roverControl control{};
	//float x, y;					 // calc gradient and position of main vector
	static hrt_abstime no_line_time = 0;		// time variable for time since no line detected
	hrt_abstime time_diff = 0;
	static bool first_call = true;
	uint8_t num_vectors = get_num_vectors(vec1, vec2);
	printf("c %f/%f\n", (double)control.speed, (double)control.steer);

	printf("v %d/%d/%d/%d %d/%d/%d/%d\n",vec1.m_x0, vec1.m_y0, vec1.m_x1, vec1.m_y1, vec2.m_x0, vec2.m_y0, vec2.m_x1, vec2.m_y1);

	float vec1Length=sqrt((vec1.m_x0-vec1.m_x1)*(vec1.m_x0-vec1.m_x1) +(vec1.m_y0-vec1.m_y1)*(vec1.m_y0-vec1.m_y1));

	float vec2Length=sqrt((vec2.m_x0-vec2.m_x1)*(vec2.m_x0-vec2.m_x1) +(vec2.m_y0-vec2.m_y1)*(vec2.m_y0-vec2.m_y1));
	
	float eps = 25.0f;

	//steer positiv stanga, negativ dreapta

	//vec1 stanga

	// if(vec1.m_x0 > vec2.m_x0)
	// {
	// 	Vector aux=vec1;
	// 	vec1=vec2;
	// 	vec2=aux;
	// }

	switch (num_vectors)
	{
	case 0:
	
		if(first_call){
			no_line_time = hrt_absolute_time();
			first_call = false;
		}else{
			time_diff = hrt_elapsed_time_atomic(&no_line_time);
			control.steer = 0.0f;
			if(time_diff > 1000){
				/* Stopping if no vector is available */
				control.steer = 0.0f;
				control.speed = SPEED_STOP;
				}
			}
		break;
	
	case 1:

	first_call = true;
		if(vec1.m_x0 < window_center)
		{
			control.speed=SPEED_NORMAL;   //la dreapta
			control.steer=-0.6f;    //era  0.4f 
		}
		else
		{
			control.speed=SPEED_NORMAL;   //la dreapta
			control.steer=0.6f;    
		}
		

		break;

	case 2:
		first_call = true;
		if(abs(vec1Length-vec2Length)>eps)
			{
				if(vec1Length>vec2Length)
				{
					control.speed=SPEED_NORMAL;  //STOP
					control.steer=-0.6f;
				}
				else
				{
					control.speed=SPEED_NORMAL;  //STOP
					control.steer=0.6f;
				}
				return control;
			}
		if(vec1.m_x1< window_center && vec2.m_x1 > window_center)
			{
				control.speed=SPEED_FAST;   //in centru
				control.steer=0.0f;
			}
			else
			{
				if(vec1.m_x1>= window_center)
					{
						control.speed=SPEED_SLOW;   //la stanga   
						control.steer=0.3f;   //0.6
					}
				else
					{
						control.speed=SPEED_SLOW;   //la dreapta
						control.steer=-0.3f;   //0.6
					} 
			}
		break;
	}

	//main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
	//control.steer = (float)(main_vec.m_x1 - window_center) / (float)frameWidth;

	return control;
}