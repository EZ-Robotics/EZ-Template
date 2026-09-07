// util.hpp includes <bits/stdc++.h>, GCC's non-standard umbrella header --
// not provided by other compilers (e.g. clang, used to develop this test
// suite locally). Since -I test/stub is searched before the real system
// headers, this is found first and just pulls in the specific standard
// headers util.hpp/util.cpp actually use, so the test suite also builds on a
// toolchain that doesn't ship the GCC umbrella header.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
