// Copyright 2016 Olivier Gillet.
//
// Author: Olivier Gillet (pichenettes@mutable-instruments.net)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// 2 variable shape oscillators with sync, FM and crossfading.

#include "plaits/dsp/engine/virtual_analog_mod_square.h"

#include <algorithm>

#include "stmlib/dsp/parameter_interpolator.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void VirtualAnalogModSquare::Init(BufferAllocator* allocator) {
  square1_.Init();
  square2_.Init();
  saw1_.Init();
  saw2_.Init();

  overdrive_.Init();

  auxiliary_amount_ = 0.0f;
  xmod_amount_ = 0.0f;

  temp_buffer_ = allocator->Allocate<float>(kMaxBlockSize);
}

void VirtualAnalogModSquare::Reset() {

}

const float intervals[5] = {
  0.0f, 7.01f, 12.01f, 19.01f, 24.01f
};

inline float Squash(float x) {
  return x * x * (3.0f - 2.0f * x);
}

float VirtualAnalogModSquare::ComputeDetuning(float detune) const {
  detune = 2.05f * detune - 1.025f;
  CONSTRAIN(detune, -1.0f, 1.0f);

  float sign = detune < 0.0f ? -1.0f : 1.0f;
  detune = detune * sign * 3.9999f;
  MAKE_INTEGRAL_FRACTIONAL(detune);

  float a = intervals[detune_integral];
  float b = intervals[detune_integral + 1];
  return (a + (b - a) * Squash(Squash(detune_fractional))) * sign;
}


void VirtualAnalogModSquare::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {

#if VA_VARIANT == 2

  const float auxiliary_detune = ComputeDetuning(parameters.harmonics);
  const float primary_f = NoteToFrequency(parameters.note);
  const float auxiliary_f = NoteToFrequency(parameters.note + auxiliary_detune);

  // control for overdrive --> MORPH knob

  // Render double saw to AUX

  /*float saw_pw = parameters.timbre < 0.5f
      ? parameters.timbre + 0.5f
      : 1.0f - (parameters.timbre - 0.5f) * 2.0f;
  saw_pw *= 1.1f;
  */

  float saw_pw = parameters.timbre / 2.0f * 1.1f;

  CONSTRAIN(saw_pw, 0.005f, 1.0f);

  float saw_shape = 10.0f - 21.0f * parameters.timbre;
  CONSTRAIN(saw_shape, 0.0f, 1.0f);

  float saw_gain = 8.0f * (1.0f - parameters.timbre);
  CONSTRAIN(saw_gain, 0.02f, 1.0f);

  // Render saws to AUX
  saw1_.Render(primary_f, saw_pw, saw_shape, out, size);
  saw2_.Render(auxiliary_f, saw_pw, saw_shape, aux, size);

  /*float saw_norm = 1.0f / (std::max(saw_gain, saw_gain));

  ParameterInterpolator square_gain_modulation_aux(
      &auxiliary_amount_,
      saw_gain * 0.5f * saw_norm,
      size);

  ParameterInterpolator saw_gain_modulation_aux(
      &xmod_amount_,
      saw_gain * 0.5f * saw_norm,
      size);
  */

  for (size_t i = 0; i < size; ++i) {
    aux[i] = aux[i] * 0.5f + out[i] * 0.5f;
  }

  const float aux_drive = parameters.morph * 3.0;

  overdrive_.Process(
    0.5f + 0.5f * aux_drive,
    aux,
    size);

  // Render double square to OUT.
  // controls for square waves -- TIMBRE knob

  float square_pw = parameters.timbre * 1.1f / 2.0f;
  CONSTRAIN(square_pw, 0.005f, 0.5f);

  //float square_gain = min(parameters.timbre * 8.0f, 1.0f);

  // render

  square1_.Render<false>(
      primary_f, primary_f, square_pw, 1.0f, temp_buffer_, size);

  square2_.Render<false>(
      auxiliary_f, auxiliary_f, square_pw, 1.0f, out, size);

  /*
  float square_norm = 1.0f / (std::max(square_gain, square_gain));

  ParameterInterpolator square_gain_modulation(
      &auxiliary_amount_,
      square_gain * 0.5f * square_norm,
      size);

  ParameterInterpolator saw_gain_modulation(
      &xmod_amount_,
      square_gain * 0.5f * square_norm,
      size);

  for (size_t i = 0; i < size; ++i) {
    out[i] = out[i] * saw_gain_modulation.Next() + \
        square_gain_modulation.Next() * temp_buffer_[i];
  }
  */

  for (size_t i = 0; i < size; ++i) {
    out[i] = out[i] * 0.5f + temp_buffer_[i] * 0.5f;
  }

  const float out_drive = parameters.morph * 3.0;

  overdrive_.Process(
    0.5f + 0.5f * out_drive,
    out,
    size);

#endif  // VA_VARIANT values

}

}  // namespace plaits
