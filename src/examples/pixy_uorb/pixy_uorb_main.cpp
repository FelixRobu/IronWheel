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
 * @file pixy_uorb_main.cpp
 * Pixy Vector publisher
 *
 * @author Landon Haugh
 */

#include "pixy_uorb_main.h"

using namespace matrix;

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit_uorb = false;
bool threadIsRunning_uorb = false;

// void normalizeVector(Pixy2 &pixy)
// {
// 	float alpha = 0.2;
// 	int x = 79 / 2;
// 	for(uint8_t i = 0; i < pixy.line.numVectors; ++i)
// 	{
// 		int x0 = pixy.line.vectors[i].m_x0;
// 		int x1 = pixy.line.vectors[i].m_x1;
// 		x0 -= x;
// 		x1 -= x;
// 		x0 /= (1.0f + alpha * pixy.line.vectors[i].m_y0);
// 		x1 /= (1.0f + alpha * pixy.line.vectors[i].m_y1);
// 		x0 += x;
// 		x1 += x;
// 		pixy.line.vectors[i].m_x0 = x0;
// 		pixy.line.vectors[i].m_x1 = x1;
// 	}
// }

int pixy_uorb_thread_main(int argc, char **argv)
{
	threadIsRunning_uorb = true;

	/* Publication of uORB messages */
	uORB::Publication<pixy_vector_s> _pixy_vector_pub{ORB_ID(pixy_vector)};
	struct pixy_vector_s _pixy_vector;

	/* Pixy2 Instance */
	Pixy2 pixy;
	bool wait = 1;		// needed for waiting for valid data
	usleep(5000);		// give pixy time to init

	// Make sure pixy is ready
	if (pixy.init() == 0) {

		// Print Pixy details to confirm Pixy is publishing data over i2c
		pixy.getVersion();
		pixy.version->print();
		usleep(1000);

		// Loop indefinitely and publish vector data
		while (1) {

			pixy.line.getAllFeatures(LINE_VECTOR,wait);
			if(pixy.line.numVectors) {
				_pixy_vector.m0_x0 = pixy.line.vectors[0].m_x0;
				_pixy_vector.m0_x1 = pixy.line.vectors[0].m_x1;
				_pixy_vector.m0_y0 = pixy.line.vectors[0].m_y0;
				_pixy_vector.m0_y1 = pixy.line.vectors[0].m_y1;
				printf("\n#1 %d %d %d %d \n#2", pixy.line.vectors[0].m_x0, pixy.line.vectors[0].m_x1, pixy.line.vectors[0].m_y0, pixy.line.vectors[0].m_y1);
				if(pixy.line.numVectors > 1) {
					_pixy_vector.m1_x0 = pixy.line.vectors[1].m_x0;
					_pixy_vector.m1_x1 = pixy.line.vectors[1].m_x1;
					_pixy_vector.m1_y0 = pixy.line.vectors[1].m_y0;
					_pixy_vector.m1_y1 = pixy.line.vectors[1].m_y1;
				} else {
					_pixy_vector.m1_x0 = 0;
					_pixy_vector.m1_x1 = 0;
					_pixy_vector.m1_y0 = 0;
					_pixy_vector.m1_y1 = 0;
				}
				//normalizeVector(pixy);
				//_pixy_vector.timestamp = hrt_absolute_time();
				//_pixy_vector_pub.publish(_pixy_vector);
			}
			else{
				_pixy_vector.m0_x0 = 0;
				_pixy_vector.m0_x1 = 0;
				_pixy_vector.m0_y0 = 0;
				_pixy_vector.m0_y1 = 0;
				
				_pixy_vector.m1_x0 = 0;
				_pixy_vector.m1_x1 = 0;
				_pixy_vector.m1_y0 = 0;
				_pixy_vector.m1_y1 = 0;
			}
			_pixy_vector.timestamp = hrt_absolute_time();
			_pixy_vector_pub.publish(_pixy_vector);


			if (threadShouldExit_uorb) {
				threadIsRunning_uorb = false;
				PX4_INFO("Exit Pixy uORB Thread!\n");
				return 1;
			}
		}

	}
	return 0;
}


extern "C" __EXPORT int pixy_uorb_main(int argc, char *argv[]);
int pixy_uorb_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: pixy_uorb {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (threadIsRunning_uorb) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		threadShouldExit_uorb = false;
		daemon_task = px4_task_spawn_cmd("pixy_uorb",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 pixy_uorb_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		threadShouldExit_uorb = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (threadIsRunning_uorb) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	PX4_WARN("usage: pixy_uorb {start|stop|status}\n");
	return 1;
}