/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
 *
 * \copyright Copyright 2021 Chalmers
 *
 * #### License
 * All rights reserved.
 *
 * Software License Agreement (BSD License 2.0)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

// TODO: Nothing to do here!

// Robot2W control class
#include <robot_2w_cam/Robot2WCam.hpp>

#include <exception>

int main(int argc, char* argv[])
{
  try
  {
    // Initialize the ros node
    rclcpp::init(argc, argv);

    // We created an object of the class Robot2WCam and run the spin
    // This will trigger the threads in the node
    rclcpp::spin(std::make_shared<robot_2w_examples::Robot2WCam>());

    // Stop the ros node
    rclcpp::shutdown();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  return 0;
}