/* 
 *  Squeezelite - lightweight headless squeezebox emulator
 *
 *  (c) Adrian Smith 2012-2015, triode1@btinternet.com
 *      Ralph Irving 2015-2017, ralph_irving@hotmail.com
 *		Philippe_44	 2020, philippe_44@outloook.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "squeezelite.h"

#define VISUEXPORT_SIZE	2048
#define LEDVISUEXPORT_SIZE	128
// separate buffers for oled and rgb led vu meters
#define NUM_BUFFERS 2 

EXT_BSS struct visu_export_s visu_export, led_visu_export;
static struct visu_export_s *visu[] = {&visu_export,&led_visu_export};

static log_level loglevel = lINFO;

void output_visu_export(s16_t *frames, frames_t out_frames, u32_t rate, bool silence, u32_t gain) {
	
	// no data to process
	if (silence) {
		visu[0]->running = false;
		visu[1]->running = false;
		return;
	}

	for (int i = NUM_BUFFERS; --i >= 0;)	{
		struct visu_export_s *v = visu[i];
		// do not block, try to stuff data put wait for consumer to have used them
		if (!pthread_mutex_trylock(&v->mutex)) {
			// don't mix sample rates
			if (v->rate != rate) {v->level = 0;}
			// stuff buffer up and wait for consumer to read it (should reset level)
			if (v->level < v->size)	{
				u32_t space = min(v->size - v->level, out_frames * 2) * 2;
				memcpy(v->buffer + v->level, frames, space);

				v->level += space / 2;
				v->running = true;
				v->rate = rate ? rate : 44100;
				v->gain = gain;
			}
			// mutex must be released
			pthread_mutex_unlock(&v->mutex);
		}
	}
}

void output_visu_close(void) {
	for (int i = NUM_BUFFERS; --i >= 0;)	{
		struct visu_export_s *v = visu[i];
		pthread_mutex_lock(&v->mutex);
		v->running = false;
		free(v->buffer);
		pthread_mutex_unlock(&v->mutex);
	}
}

void output_visu_init(log_level level) {
	loglevel = level;
	for (int i = NUM_BUFFERS; --i >= 0;)	{
		struct visu_export_s *v = visu[i];

		pthread_mutex_init(&v->mutex, NULL);
		v->size = i?LEDVISUEXPORT_SIZE:VISUEXPORT_SIZE;
		v->running = false;
		v->rate = 44100;
		v->buffer = malloc(v->size * sizeof(s16_t) * 2);
		LOG_INFO("Initialize VISUEXPORT %u 16 bits samples", v->size);
	}
}


