/*
 * libalsa output driver. This file is part of Shairport.
 * Copyright (c) Muffinman, Skaman 2013
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#define ALSA_PCM_NEW_HW_PARAMS_API

#include "audio.h"
#include "common.h"
#include <math.h>
#include <memory.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

#include <atomic>
#include <AudioToolbox/AudioToolbox.h>


#ifndef COMPILE_FOR_OSX
#error Does not work on platforms other than macos and iOS
#endif

static void help(void);
static int init(int argc, char **argv);
static void deinit(void);
static void start(int i_sample_rate, int i_sample_format);
static void play(short buf[], int samples);
static void stop(void);
static void flush(void);
int delay(long *the_delay);
void do_mute(int request);

static void volume(double vol);

static void parameters(audio_parameters *info);
static void mute(int do_mute);
static double set_volume;
static int output_method_signalled = 0;

audio_output audio_atbox = {
    .name = "atbox",
    .help = &help,
    .init = &init,
    .deinit = &deinit,
    .start = &start,
    .stop = &stop,
    .flush = &flush,
    .delay = &delay,
    .play = &play,
    .mute = &mute,   // a function will be provided if it can, and is allowed to, do hardware mute
    .volume = NULL, // a function will be provided if it can do hardware volume
    .parameters = &parameters};

static pthread_mutex_t atbox_mutex = PTHREAD_MUTEX_INITIALIZER;

static double output_latency = 0;
static int output_sample_rate;

static AUAudioUnit* audio_unit = nil;
static AVAudioOutputNode* output_node = nil;

size_t frame_size;
size_t audio_size, audio_occupancy;
char *audio_buffer, *audio_buffer_end, *audio_queue_start, *audio_queue_end;


static int volume_set_request = 0;       // set when an external request is made to set the volume.
int mute_request_pending = 0;            //  set when an external request is made to mute or unmute.
int overriding_mute_state_requested = 0; // 1 = mute; 0 = unmute requested

static void help(void) {
  printf("    -d output-device    set the output device [default*|...]\n"
         "    -m mixer-device     set the mixer device ['output-device'*|...]\n"
         "    -c mixer-control    set the mixer control [Master*|...]\n"
         "    -i mixer-index      set the mixer index [0*|...]\n"
         "    *) default option\n");
}

static void fillBuffer(AudioUnitRenderActionFlags *actionFlags, const AudioTimeStamp *timestamp, 
                       AUAudioFrameCount frameCount, NSInteger inputBusNumber, 
                       AudioBufferList *inputData) {

  size_t nBuffers = inputDataPtr->mNumberBuffers;
  if (nBuffers == 0) {
    return;
  }
  AudioBuffer aBuffer = inputDataPtr->mBuffers[0];

  void* buffer = aBuffer.mData;
  UInt32 buffer_size = aBuffer.mDataByteSize


  size_t bytes_to_transfer = frameCount * frame_size;
  size_t bytes_transferred = 0;

  // bytes we can transfer will never be greater than the bytes available
  if (bytes_to_transfer > buffer_size) {
    bytes_to_transfer = buffer_size;
    // FIXME should never happen
  }

  while ((bytes_to_transfer > 0) && (audio_occupancy > 0)) {
    size_t bytes_we_can_transfer = bytes_to_transfer;
    if (audio_occupancy < bytes_we_can_transfer) {
      // debug(1, "Underflow? We have %d bytes but we are asked for %d bytes", audio_occupancy,
      //      bytes_we_can_transfer);
      //pa_stream_cork(stream, 1, stream_success_cb, mainloop);
      // debug(1, "Corked");
     

      bytes_we_can_transfer = audio_occupancy;
    }

    void* dst_buffer = buffer + bytes_transferred;
    if (bytes_we_can_transfer <= (audio_buffer_end - audio_queue_start)) {

      // the bytes are all in a row in the audo buffer
      memcpy(dst_buffer, audio_queue_start, bytes_we_can_transfer);
      audio_queue_start += bytes_we_can_transfer;
      // lock
      pthread_mutex_lock(&atbox_mutex);
      audio_occupancy -= bytes_we_can_transfer;
      pthread_mutex_unlock(&atbox_mutex);
      // unlock
      //pa_stream_write(stream, buffer, bytes_we_can_transfer, NULL, 0LL, PA_SEEK_RELATIVE);
      bytes_transferred += bytes_we_can_transfer;
    } else {
      // the bytes are in two places in the audio buffer
      size_t first_portion_to_write = audio_buffer_end - audio_queue_start;
      if (first_portion_to_write != 0) {
        memcpy(dst_buffer, audio_queue_start, first_portion_to_write);
      }

      dst_buffer = dst_buffer + first_portion_to_write;
      memcpy(dst_buffer, audio_buffer, bytes_we_can_transfer - first_portion_to_write);

      //pa_stream_write(stream, buffer, bytes_we_can_transfer, NULL, 0LL, PA_SEEK_RELATIVE);
      bytes_transferred += bytes_we_can_transfer;
      audio_queue_start = audio_buffer + bytes_we_can_transfer - first_portion_to_write;
      // lock
      pthread_mutex_lock(&atbox_mutex);
      audio_occupancy -= bytes_we_can_transfer;
      pthread_mutex_unlock(&atbox_mutex);
      // unlock
    }
    bytes_to_transfer -= bytes_we_can_transfer;
    // debug(1,"audio_toq is %llx",audio_toq);
  }

  size_t frames_transf = bytes_transferred / frame_size;
  if (frames_transf < frameCount) {
    memset(buffer + bytes_transferred, 0, (frameCount - frames_transf) * frame_size);
  }

}



static int init(int argc, char **argv) {
  pthread_mutex_lock(&atbox_mutex);
  // debug(2,"audio_alsa init called.");
  const char *str;
  int value;
  double dvalue;

  config.alsa_use_playback_switch_for_mute = 0; // don't use it by default

  // get settings from settings file first, allow them to be overridden by
  // command line options

  // do the "general" audio  options. Note, these options are in the "general" stanza!
  parse_general_audio_options();

  AVAudioSession *audioSession = [AVAudioSession sharedInstance];
  NSError* error = nil;
  if (!audioSession.isOutputAvailable) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }
  [audioSession setCategory:AVAudioSessionCategoryPlayAndRecord error:&error];
  if (error != nil) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }
  [audioSession setPreferredIOBufferDuration:0.15 error:&error];
  if (error != nil) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }

  config.audio_backend_latency_offset = 0;
  config.audio_backend_buffer_desired_length = audioSession.IOBufferDuration;
  config.output_format = SPS_FORMAT_S16;
  config.output_rate = 44100;

  [audioSession setPreferredSampleRate:44100.0 error:error];
  if (error != nil) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }

  [audioSession setActive:YES error:&error];
  if(error) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }

  output_latency = audioSession.outputLatency;
  output_sample_rate = audioSession.sampleRate;
  //audioSession.outputLatency
    /*config.no_sync = 0;
  config.output_format
  config.output_rate
  no_mmap*/

  AudioComponentDescription* componentDescription = [AudioComponentDescription new];
  componentDescription.componentType = kAudioUnitType_Output;
  componentDescription.componentSubType = kAudioUnitSubType_DefaultOutput;
  componentDescription.componentManufacturer = kAudioUnitManufacturer_Apple;
  componentDescription.componentFlags = 0;
  componentDescription.componentFlagsMask = 0;
  
  AVAudioUnitComponentManager* mngr = [AVAudioUnitComponentManager shared];
	NSArray<AVAudioUnitComponent *>* units = [mngr componentsMatchingDescription: componentDescription];
	if (units.count == 0) {
    pthread_mutex_unlock(&atbox_mutex);
    die("Did not find a single output device");
  }
  AVAudioUnitComponent* component = units[0];
  NSError* error = nil;
  audio_unit = [[AUAudioUnit alloc] initWithComponentDescription:component.audioComponentDescription 
                                                         options:kAudioComponentInstantiation_LoadInProcess 
                                                           error:&error];
  if (!audio_unit) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }


  AUAudioUnitBus* bus0 = auAudioUnit.inputBusses[0]
  AVAudioFormat* audioFormat = [[AVAudioFormat alloc] initWithCommonFormat:AVAudioPCMFormatInt16 
                                                                sampleRate:44100.0 
                                                                channels:2 
                                                                interleaved:true];
  [bus0 setFormat:audioFormat error:error];
  if (error != nil) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }

  //output_node = audo_engine.outputNode
  //double latency = output_node.presentationLatency;
  //double ddd = output_node.outputPresentationLatency;
  //[outputNode inputFormatForBus:0];
  //AVAudioFormat *format = [input outputFormatForBus: 0];

  /*if (config.cfg != NULL) {

    // Get the Output Device Name.
    if (config_lookup_string(config.cfg, "alsa.output_device", &str)) {
      alsa_out_dev = (char *)str;
    }
  }*/


  // Four seconds buffer -- should be plenty
  frame_size = (sizeof(uint16_t) * 2);
  audio_size = config.output_rate * 4 * frame_size;

  // allocate space for the audio buffer
  audio_buffer = malloc(audio_size);
  if (audio_buffer == NULL) {
    pthread_mutex_unlock(&atbox_mutex);
    die("Can't allocate %d bytes for AV buffer.", audio_size);
  }
  audio_queue_start = audio_queue_end = audio_buffer;
  audio_buffer_end = audio_buffer + audio_size;
  audio_occupancy = 0;

  //debug(1, "alsa output device name is \"%s\".", alsa_out_dev);

  pthread_mutex_unlock(&atbox_mutex);
  return 0;
}

static void deinit(void) {
  // debug(2,"audio_alsa deinit called.");
  stop();
}

static void start(int i_sample_rate, int i_sample_format) {
  pthread_mutex_lock(&atbox_mutex);
  audio_unit.isOutputEnabled = true

  NSError* error = nil;
  [audio_unit allocateRenderResourcesAndReturnError:&error];//  v2 AudioUnitInitialize()
  if (error != nil) {
    pthread_mutex_unlock(&atbox_mutex);
    die("Can't allocate render resources.");
  }
  [audio_unit startHardwareAndReturnError:&error];
  if (error != nil) {
    pthread_mutex_unlock(&atbox_mutex);
    die("Can't start hardware.");
  }
}

static void stop(void) {
  [audio_unit stopHardware];
  NSError* error = nil;
  [audioSession setActive:NO error:&error];
  if(error) {
    pthread_mutex_unlock(&atbox_mutex);
    die("initializing audio output failed");
  }

  // debug(2,"audio_alsa stop called.");
  // when we want to stop, we want the alsa device
  // to be closed immediately -- we may even be killing the thread, so we
  // don't wish to wait
  // so we should flush first
  flush(); // flush will also close the device
           // close_alsa_device();
}

static int delay(long *the_delay) {
  AVAudioSession *audioSession = [AVAudioSession sharedInstance];
  double latency = audioSession.outputLatency;
  *the_delay = (audio_occupancy / (2 * 2)) + (latency * output_sample_rate) / 1000000;
  return 0;
}

static void play(short buf[], int samples) {
  // debug(1,"pa_play of %d samples.",samples);
  char *bbuf = (char *)buf;
  // copy the samples into the queue
  size_t bytes_to_transfer = samples * frame_size;
  size_t space_to_end_of_buffer = audio_buffer_end - audio_queue_end;
  if (space_to_end_of_buffer >= bytes_to_transfer) {
    memcpy(audio_queue_end, bbuf, bytes_to_transfer);
    audio_occupancy += bytes_to_transfer;
    pthread_mutex_lock(&buffer_mutex);
    audio_queue_end += bytes_to_transfer;
    pthread_mutex_unlock(&buffer_mutex);
  } else { // wrap around
    memcpy(audio_queue_end, bbuf, space_to_end_of_buffer);
    bbuf += space_to_end_of_buffer;
    memcpy(audio_buffer, bbuf, bytes_to_transfer - space_to_end_of_buffer);
    pthread_mutex_lock(&buffer_mutex);
    audio_occupancy += bytes_to_transfer;
    pthread_mutex_unlock(&buffer_mutex);
    audio_queue_end = audio_buffer + bytes_to_transfer - space_to_end_of_buffer;
  }
  /*if ((audio_occupancy >= 11025 * 2 * 2) && (pa_stream_is_corked(stream))) {
    // debug(1,"Uncorked");
    pa_threaded_mainloop_lock(mainloop);
    pa_stream_cork(stream, 0, stream_success_cb, mainloop);
    pa_threaded_mainloop_unlock(mainloop);
  }*/
}

static void flush(void) {
  // debug(2,"audio_alsa flush called.");
  pthread_mutex_lock(&atbox_mutex);
  
  audio_queue_start = audio_queue_end = audio_buffer;
  audio_buffer_end = audio_buffer + audio_size;
  audio_occupancy = 0;

  pthread_mutex_unlock(&atbox_mutex);
}

static void parameters(audio_parameters *info) {
  info->minimum_volume_dB = 10;
  info->maximum_volume_dB = 60;
}

void volume(double vol) {
  pthread_mutex_lock(&atbox_mutex);
  
  pthread_mutex_unlock(&atbox_mutex);
}

/*
static void linear_volume(double vol) {
  debug(2, "Setting linear volume to %f.", vol);
  set_volume = vol;
  if (hardware_mixer && alsa_mix_handle) {
    double linear_volume = pow(10, vol);
    // debug(1,"Linear volume is %f.",linear_volume);
    long int_vol = alsa_mix_minv + (alsa_mix_maxv - alsa_mix_minv) * linear_volume;
    // debug(1,"Setting volume to %ld, for volume input of %f.",int_vol,vol);
    if (alsa_mix_handle) {
      if (snd_mixer_selem_set_playback_volume_all(alsa_mix_elem, int_vol) != 0)
        die("Failed to set playback volume");

    }
  }
}
*/

static void mute(int mute_state_requested) {
  // debug(1,"External Mute Request: %d",mute_state_requested);
  pthread_mutex_lock(&atbox_mutex);
  mute_request_pending = 1;
  overriding_mute_state_requested = mute_state_requested;
  do_mute(mute_state_requested);
  pthread_mutex_unlock(&atbox_mutex);
}

void do_mute(int mute_state_requested) {

  // if a mute is requested now, then
  // 	if an external mute request is in place, leave everything muted
  //  otherwise, if an external mute request is pending, action it
  //  otherwise, action the do_mute request

  int local_mute_state_requested =
      overriding_mute_state_requested; // go with whatever was asked by the external "mute" call

  // The mute state requested will be actioned unless mute_request_pending is set
  // If it is set, then that the pending request will be actioned.
  // If the hardware isn't there, or we are not allowed to use it, nothing will be done
  // The caller must have the alsa mutex

  if (config.alsa_use_playback_switch_for_mute == 1) {
    if (mute_request_pending == 0)
      local_mute_state_requested = mute_state_requested;
    if (open_mixer()) {
      if (local_mute_state_requested) {
        // debug(1,"Playback Switch mute actually done");
        snd_mixer_selem_set_playback_switch_all(alsa_mix_elem, 0);
      } else if (overriding_mute_state_requested == 0) {
        // debug(1,"Playback Switch unmute actually done");
        snd_mixer_selem_set_playback_switch_all(alsa_mix_elem, 1);
      }
      close_mixer();
    }
  }
  mute_request_pending = 0;
}
