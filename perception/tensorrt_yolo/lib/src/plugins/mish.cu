// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * MIT License

 * Copyright (c) 2019-2020 Wang Xinyu

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <mish.hpp>

#include <cuda_runtime_api.h>
#include <stdio.h>

namespace yolo
{
__device__ float mish(float x)
{
  float e = __expf(x);
  float n = e * e + 2 * e;
  if (x <= -0.6f) return x * __fdividef(n, n + 2);

  return x - 2 * __fdividef(x, n + 2);
}
__device__ float mish_standard(float x)
{
  return x * tanh(log1p(exp(x)));
}
template <typename T, unsigned TPB>
__global__ void mishKernel(const T * input, T * output, int num_elem)
{
  int idx = threadIdx.x + TPB * blockIdx.x;
  if (idx >= num_elem) return;
  output[idx] = mish(input[idx]);
}

int mish(cudaStream_t stream, const float * input, float * output, int n)
{
  constexpr int blockSize = 256;
  const int gridSize = (n + blockSize - 1) / blockSize;
  mishKernel<float, blockSize><<<gridSize, blockSize, 0, stream>>>(input, output, n);
  return 0;
}

}  // namespace yolo
