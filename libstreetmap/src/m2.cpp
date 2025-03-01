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
#include <unordered_set>

// function declarations
void drawFeatures(ezgl::renderer *g);
void drawRoads(ezgl::renderer *g, double zoomLevel);
double x_from_lon(double lon);
double y_from_lat(double lat);
void loadHighway();
int classify_road(OSMID way_id);
void calculate_map_bound(double &min_lat, double &max_lat, double &min_lon, double &max_lon);
void drawStreetSegments(ezgl::renderer *g, int priority);
void drawStreetNames(ezgl::renderer *g, double zoomLevel);
void load_road_data();
void draw_main_canvas(ezgl::renderer *g);
void setInterface(ezgl::application &application);

// global variables
std::vector<std::vector<ezgl::point2d>> roads;  // Store each road as a list of points
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

//output the road based on the classified osm type
void drawStreetSegments(ezgl::renderer *g, int priority){
   for(size_t i = 0; i < roads.size(); i++){
      int roadType = road_types[i];
      if(priority == 3 && roadType < 3){
         continue;
      }
      if(priority == 2 && roadType < 2){
         continue;
      }

      if(roadType == 3){
         g->set_color(255, 140, 0);
         g->set_line_width(5);
      }
      else if(roadType == 2){
         g->set_color(156, 150, 150);
         g->set_line_width(4);
      }
      else{
         g->set_color(ezgl::WHITE);
         g->set_line_width(3);
      }
      for(size_t j = 0; j < roads[i].size() - 1; j++){
         g->draw_line(roads[i][j], roads[i][j+1]);
      }
   }
}

void drawStreetNames(ezgl::renderer *g, double zoomLevel)
{
   if (zoomLevel >= 5000)
   {
      return; // Only draw names when zoomed in
   }

   std::unordered_set<std::string> drawnNames;

   for (size_t i = 0; i < roads.size(); i++)
   {
      StreetSegmentInfo segInfo = getStreetSegmentInfo(i);
      std::string streetName = getStreetName(segInfo.streetID);

      if (streetName.empty() || drawnNames.find(streetName) != drawnNames.end())
      {
         continue; // Skip if no name or already drawn
      }

      // Find the longest segment for text placement
      size_t maxIdx = 0;
      double maxLen = 0;

      for (size_t j = 0; j < roads[i].size() - 1; j++)
      {
         double length = sqrt(pow(roads[i][j + 1].x - roads[i][j].x, 2) +
                              pow(roads[i][j + 1].y - roads[i][j].y, 2));
         if (length > maxLen)
         {
            maxLen = length;
            maxIdx = j;
         }
      }

      ezgl::point2d start = roads[i][maxIdx];
      ezgl::point2d end = roads[i][maxIdx + 1];
      ezgl::point2d midPoint((start.x + end.x) / 2, (start.y + end.y) / 2);

      g->set_font_size(10);
      g->set_color(ezgl::BLACK);
      g->draw_text(midPoint, streetName);

      drawnNames.insert(streetName); // Ensure we don't draw the same name multiple times
   }
}

// Load Roads and Convert to ezgl::point2d
void load_road_data() {
    roads.clear();
    road_types.clear();
    oneWayRoad.clear();

    for (int i = 0; i < getNumStreetSegments(); i++) {
        StreetSegmentInfo seg = getStreetSegmentInfo(i);
        std::vector<ezgl::point2d> road_points;

        // Convert intersection positions
        LatLon start = getIntersectionPosition(seg.from);
        LatLon end = getIntersectionPosition(seg.to);
        road_points.push_back({x_from_lon(start.longitude()), y_from_lat(start.latitude())});

        // Convert curve points (if any)
        for (int j = 0; j < seg.numCurvePoints; j++) {
            LatLon curve = getStreetSegmentCurvePoint(i, j);
            road_points.push_back({x_from_lon(curve.longitude()), y_from_lat(curve.latitude())});
        }

        // Add end position
        road_points.push_back({x_from_lon(end.longitude()), y_from_lat(end.latitude())});

        roads.push_back(road_points);
        road_types.push_back(classify_road(seg.wayOSMID));  // Classify road type
    }
}

// Draw Roads Based on Classification
void draw_main_canvas(ezgl::renderer *g)
{
   g->set_color(220, 220, 220);
   g->fill_rectangle(g->get_visible_world());
   
   double zoomLevel = g -> get_visible_screen().width();
    drawFeatures(g);
    drawRoads(g, zoomLevel);    

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

void drawRoads(ezgl::renderer *g, double zoomLevel) {
   if(zoomLevel <= 15000){
      drawStreetSegments(g, 1);
   }
   else if(zoomLevel <= 70000){
      drawStreetSegments(g, 2);
   }
   else{
      drawStreetSegments(g, 3);
   }

   if(zoomLevel < 5000){
      drawStreetNames(g, zoomLevel);
   }
}