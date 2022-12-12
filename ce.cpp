/*{ Copyright © 2022 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH
                     Matthias Kretz <m.kretz@gsi.de>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the names of contributing organizations nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
}*/

#include "subscript.h"
#include <vector>

void g(auto...);

constexpr auto plusone = std::views::transform([](float x) {
   return x + 1;
});

void f(std::vector<float> data)
{
  g(data[1u, Const< 4>]);
  g(data[1u, Const< -4>]);
  g(data[-1, Const< -4>]);
  //g(data[Const< -1>, Const< -4>]); // ERROR: index range results in negative size
  g(data[Const< -4>, Const< -1>]);
  //g(data[Const< -1>, Const<4>]); // ERROR: index range exceeds bounds
  g(data[1, -4]);
  g((data | plusone)[1, 5]);
}

int test0()
{
  const std::array<int, 4> l = {1, 2, 3, 4};
  auto l2 = l | plusone;
  int aggr = 0;
  for (int x : l2[-2, Const<2>])
    aggr += x;
  return aggr;
}
