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
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <ezgl/application.hpp>
#include <ezgl/graphics.hpp>
#include <ezgl/rectangle.hpp>
#include <vector>
#include <iostream>
#include <unordered_map>


void drawFeatures(ezgl::renderer *g);



std::vector<std::pair<ezgl::point2d, ezgl::point2d>> roads;
std::vector<int> road_types;
std::vector<bool> oneWayRoad;
std::unordered_map<OSMID, std::string> osmHighway;

double avgLat;

// Convert Latitude/Longitude to X/Y using Equirectangular Projection
double x_from_lon(double lon) {
    return lon * kDegreeToRadian * kEarthRadiusInMeters * cos(avgLat * kDegreeToRadian);
}

double y_from_lat(double lat) {
    return lat * kDegreeToRadian * kEarthRadiusInMeters;
}

void loadHighway(){
   osmHighway.clear();
   int numWays = getNumberOfWays();
   for(int i = 0; i < numWays; i++){
      const OSMWay *way = getWayByIndex(i);
      for(int j = 0; j < getTagCount(way); j++){
         std::pair<std::string, std::string> tag = getTagPair(way, j);
         if(tag.first == "highway"){
            osmHighway[way->id()] = tag.second;
            break;
         }
      }
   }
}

//differentiate road type
int classify_road(OSMID way_id){
   if(osmHighway.find(way_id) == osmHighway.end()){
      return 1;
   }
   std::string roadType = osmHighway[way_id];
   if(roadType == "motorway" || roadType == "trunk" || roadType == "expressway"){
      return 3;
   }
   else if(roadType == "primary"){
      return 2;
   }
   else if(roadType == "secondary" || roadType == "tertiary"){
      return 1;
   }
   return 0;
}



// Determine Map Boundaries
void calculate_map_bound(double &min_lat, double &max_lat, double &min_lon, double &max_lon) {
    min_lat = max_lat = getIntersectionPosition(0).latitude();
    min_lon = max_lon = getIntersectionPosition(0).longitude();

    for (int i = 1; i < getNumIntersections(); i++) {
        LatLon pos = getIntersectionPosition(i);
        min_lat = std::min(min_lat, pos.latitude());
        max_lat = std::max(max_lat, pos.latitude());
        min_lon = std::min(min_lon, pos.longitude());
        max_lon = std::max(max_lon, pos.longitude());
    }

    avgLat = (max_lat + min_lat) / 2.0;
}

// Load Roads and Convert to ezgl::point2d
void load_road_data() {
    roads.clear();
    road_types.clear();
    oneWayRoad.clear();

    for (int i = 0; i < getNumStreetSegments(); i++) {
        StreetSegmentInfo seg = getStreetSegmentInfo(i);
        LatLon start = getIntersectionPosition(seg.from);
        LatLon end = getIntersectionPosition(seg.to);

        ezgl::point2d prev_point(x_from_lon(start.longitude()), y_from_lat(start.latitude()));

        for (int j = 0; j < seg.numCurvePoints; j++) {
            LatLon curve = getStreetSegmentCurvePoint(i, j);
            ezgl::point2d curve_point(x_from_lon(curve.longitude()), y_from_lat(curve.latitude()));
            roads.emplace_back(prev_point, curve_point);
            road_types.push_back(classify_road(seg.wayOSMID));
            prev_point = curve_point; // Update previous point
        }

        ezgl::point2d end_point(x_from_lon(end.longitude()), y_from_lat(end.latitude()));
        roads.emplace_back(prev_point, end_point);
        road_types.push_back(classify_road(seg.wayOSMID));
        oneWayRoad.push_back(seg.oneWay);
    }
}

// Draw Roads Based on Classification
void draw_main_canvas(ezgl::renderer *g)
{
   g->set_color(220, 220, 220);
   g->fill_rectangle(g->get_visible_world());

   for (size_t i = 0; i < roads.size(); i++)
   {
      if (road_types[i] == 3)
      {
         g->set_color(255, 140, 0); // Orange for Highways
         g->set_line_width(6);
      }
      else if (road_types[i] == 2)
      {
         g->set_color(150, 150, 150); // Gray for Main Roads
         g->set_line_width(3);
      }
      else
      {
         g->set_color(ezgl::WHITE); // White for Secondary Roads
         g->set_line_width(2);
      }

      g->draw_line(roads[i].first, roads[i].second);
   }
   drawFeatures(g);
   //indiate the one way street
}

// Set Initial View Using LatLon Bounds
void setInterface(ezgl::application &application) {
    double min_lat, max_lat, min_lon, max_lon;
    calculate_map_bound(min_lat, max_lat, min_lon, max_lon);

    ezgl::rectangle initial_world(
        {x_from_lon(min_lon), y_from_lat(min_lat)},
        {x_from_lon(max_lon), y_from_lat(max_lat)}
    );

    application.add_canvas("MainCanvas", draw_main_canvas, initial_world);
}

// Main Draw Function
void drawMap() {
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";

    ezgl::application application(settings);
    setInterface(application);
    loadHighway();
    load_road_data();
    application.run(nullptr, nullptr, nullptr, nullptr);
}

void drawFeatures(ezgl::renderer *g) {
    for (FeatureIdx i = 0; i < getNumFeatures(); i++) {
        FeatureType type = getFeatureType(i);
        int numPoints = getNumFeaturePoints(i);

        if (numPoints < 2) continue;

        std::vector<ezgl::point2d> points;
        for (int j = 0; j < numPoints; j++) {
            LatLon latlon = getFeaturePoint(i, j);
            points.push_back(ezgl::point2d(x_from_lon(latlon.longitude()), y_from_lat(latlon.latitude())));
        }

        if (type == PARK || type == GREENSPACE) {
            g->set_color(181, 220, 159); 
        } else if (type == LAKE) {
            g->set_color(173, 216, 230); 
        } else if (type == BEACH) {
            g->set_color(238, 214, 175); 
        } else if (type == ISLAND) {
            g->set_color(205, 183, 158); 
        } else if (type == GOLFCOURSE) {
            g->set_color(119, 221, 119); 
        } else if (type == BUILDING) {
            g->set_color(169, 169, 169); 
        } else {
            g->set_color(0, 0, 0); 
        }

        if (type != RIVER && type != STREAM) {
            g->fill_poly(points);
        } else {
            g->set_color(173, 216, 230);
            g->set_line_width(2);
            for (size_t j = 0; j < points.size() - 1; j++) {
                g->draw_line(points[j], points[j + 1]);
            }
        }
    }
}