/* 
 * Copyright 2025 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "m1.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "StreetsDatabaseAPI.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <cmath>

//global variables
double avgLat = 0;

double x_from_lon(double lon){
   return lon * kDegreeToRadian * kEarthRadiusInMeters * std::cos(avgLat * kDegreeToRadian);

}

double y_from_lon(double lat){
   return lat * kDegreeToRadian * kEarthRadiusInMeters * std::cos(avgLat * kDegreeToRadian);
}

void drawMainCanvas(ezgl::renderer *g){
   g -> set_color(ezgl::WHITE);
   g -> fill_rectangle(g -> get_visible_world());
   g -> set_color(ezgl::BLACK);
   for(int i = 0; i < getNumStreetSegments(); i++){
      StreetSegmentInfo segment = getStreetSegmentInfo(i);
      ezgl::point2d from = {x_from_lon(getIntersectionPosition(segment.from).longitude()),
      y_from_lon(getIntersectionPosition(segment.from).latitude())};
      ezgl::point2d to = {x_from_lon(getIntersectionPosition(segment.to).longitude()),
      y_from_lon(getIntersectionPosition(segment.to).latitude())};


   }
}

void drawMap() {
   // Set up the ezgl graphics window and hand control to it, as shown in the 
   // ezgl example program. 
   // This function will be called by both the unit tests (ece297exercise) 
   // and your main() function in main/src/main.cpp.
   // The unit tests always call loadMap() before calling this function
   // and call closeMap() after this function returns.
   ezgl::application::settings settings;
   settings.main_ui_resource = "libstreetmap/resources/main.ui";
   settings.window_identifier = "MainWindow";
   settings.canvas_identifier = "MainCanvas";
   ezgl::application app(settings);
   ezgl::rectangle initial_world({0, 0}, {1000, 1000});
   app.add_canvas("MainCanvas", drawMainCanvas, initial_world);
   app.run(nullptr, nullptr, nullptr, nullptr);
}

