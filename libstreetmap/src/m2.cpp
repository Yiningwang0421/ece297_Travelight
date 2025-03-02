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
#include <sstream>  // Required for std::istringstream


#include <unordered_set>


// function declarations
void drawFeatures(ezgl::renderer *g, double zoomLevel);
void drawRoads(ezgl::renderer *g, double zoomLevel);
void drawPOIs(ezgl::renderer *g, double zoomLevel);
double x_from_lon(double lon);
double y_from_lat(double lat);
void loadHighway();
void loadPOIs();
int classify_road(OSMID way_id);
void calculate_map_bound(double &min_lat, double &max_lat, double &min_lon, double &max_lon);
void drawStreetSegments(ezgl::renderer *g, int priority);
void drawStreetNames(ezgl::renderer *g, double zoomLevel);
double getZoomLevel(ezgl::renderer *g, double initial_width);
ezgl::point2d findLargestInscribedRectangle(FeatureIdx feature_id);
std::string splitTextIntoLines(const std::string& text);
void drawFeatureShapes(ezgl::renderer *g, double zoomLevel);
void drawFeatureNames(ezgl::renderer *g, double zoomLevel);
void drawRiverNames(ezgl::renderer *g, double zoomLevel);
void drawPOIs(ezgl::renderer *g, double zoomLevel);
void pre_load_road_data();

void load_road_data();
void draw_main_canvas(ezgl::renderer *g);
void setInterface(ezgl::application &application);

// global variables
std::vector<std::vector<ezgl::point2d>> roads;  // Store each road as a list of points
//<a href="https://www.flaticon.com/free-icons/poi" title="poi icons">Poi icons created by Muhammad_Usman - Flaticon</a>
ezgl::surface *poi_icon;

std::vector<ezgl::point2d> POIs;
std::vector<int> road_types;
std::vector<bool> oneWayRoad;
std::vector<std::string> poiNames;
std::unordered_map<OSMID, std::string> osmHighway;




double avgLat;
double fontSize;

// Define the structure *before* using it in load_road_data()
struct RoadSegment {
    ezgl::point2d midpoint;  // The best midpoint for text placement
    double angle;            // Angle for correct text orientation
    double length;           // Road segment length (used for filtering)
    int roadType;            // 3 = Highways, 2 = Main Roads, 1 = Secondary Roads, 0 = Minor Roads
    std::string name;        // Street name
};


std::vector<RoadSegment> highways;   // RoadType 3
std::vector<RoadSegment> main_roads; // RoadType 2
std::vector<RoadSegment> secondary;  // RoadType 1
std::vector<RoadSegment> minor;      // RoadType 0

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
      return 0;
   }
   std::string roadType = osmHighway[way_id];
    if (roadType == "motorway" || roadType == "trunk" || roadType == "expressway") {
        return 3;
    } else if (roadType == "primary" || roadType == "secondary" || roadType == "tertiary") {
        return 2;
    } else if (roadType == "unclassified" || roadType == "residential" || roadType == "living_street") {
        return 1;
    } else if (roadType == "service" || roadType == "track" || roadType == "path" || 
               roadType == "footway" || roadType == "cycleway" || roadType == "bridleway" || 
               roadType == "steps" || roadType == "pedestrian") {
        return 0;
    }else
    {
        return 0;
    }
    
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
// void drawStreetSegments(ezgl::renderer *g, int priority){
//    for(size_t i = 0; i < roads.size(); i++){
//       int roadType = road_types[i];
//       if(priority == 3 && roadType < 3){
//          continue;
//       }
//       if(priority == 2 && roadType < 2){
//          continue;
//       }

//       if(roadType == 3){
//          g->set_color(255, 140, 0);
//          g->set_line_width(5);
//       }
//       else if(roadType == 2){
//          g->set_color(156, 150, 150);
//          g->set_line_width(4);
//       }
//       else{
//          g->set_color(ezgl::WHITE);
//          g->set_line_width(3);
//       }
//       for(size_t j = 0; j < roads[i].size() - 1; j++){
//          g->draw_line(roads[i][j], roads[i][j+1]);
//       }
//    }
// }

void drawStreetNames(ezgl::renderer *g, double zoomLevel) {
    if (zoomLevel < 100) return;  // Skip rendering at low zoom levels

    // **Determine which road types to display**
    std::vector<RoadSegment>* roadsToDraw = nullptr;
    if (zoomLevel >= 15000) roadsToDraw = &minor;
    else if (zoomLevel >= 4600) roadsToDraw = &secondary;
    else if (zoomLevel >= 160) roadsToDraw = &main_roads;
    else if (zoomLevel >= 100) roadsToDraw = &highways;
    
    if (!roadsToDraw) return; // No roads should be drawn

    for (const RoadSegment &road : *roadsToDraw) {
        // **Draw road name at the precomputed midpoint**
        g->set_font_size(8);
        g->set_color(ezgl::BLACK);
        g->set_text_rotation(road.angle); // Rotate text with road direction
        g->draw_text(road.midpoint, road.name);
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

//Load the plane projections of Points of Interest
void loadPOIs(){
    POIs.clear();
    for (int i=0; i<getNumPointsOfInterest(); i++){
        LatLon pos = getPOIPosition(i);
        
        ezgl::point2d projection(x_from_lon(pos.longitude()), y_from_lat(pos.latitude()));
        POIs.push_back(projection);
        
        poiNames.push_back(getPOIName(i));
    }
}

// Draw Roads Based on Classification
void draw_main_canvas(ezgl::renderer *g)
{
   g->set_color(220, 220, 220);
   g->fill_rectangle(g->get_visible_world());
   
   static double initial_width = g->get_visible_world().width();
   double zoomLevel = getZoomLevel(g, initial_width);
   fontSize = std::max(6.0, 9.0 * zoomLevel / 300.0);

    drawFeatures(g, zoomLevel);
    drawRoads(g, zoomLevel);
    if (zoomLevel > 260)
    {
        drawPOIs(g, zoomLevel);
    }
    drawStreetNames(g,zoomLevel);
   
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
    loadPOIs();
    pre_load_road_data();
    application.run(nullptr, nullptr, nullptr, nullptr);
}

void drawFeatures(ezgl::renderer *g, double zoomLevel) {
    drawFeatureShapes(g, zoomLevel);  
    drawFeatureNames(g, zoomLevel);   
    drawRiverNames(g, zoomLevel);     
}



void drawFeatureShapes(ezgl::renderer *g, double zoomLevel) {
    for (FeatureIdx i = 0; i < getNumFeatures(); i++) {
        FeatureType type = getFeatureType(i);
        int numPoints = getNumFeaturePoints(i);
        if (numPoints < 2) continue;

        // Skip drawing buildings unless zoom > 60
        if (type == BUILDING && zoomLevel < 60) continue;

        std::vector<ezgl::point2d> points;
        for (int j = 0; j < numPoints; j++) {
            LatLon latlon = getFeaturePoint(i, j);
            points.push_back(ezgl::point2d(x_from_lon(latlon.longitude()), y_from_lat(latlon.latitude())));
        }

        // Feature Colors
        if (type == PARK || type == GREENSPACE) {
            g->set_color(181, 220, 159);
        } else if (type == LAKE || type == RIVER || type == STREAM) {
            g->set_color(173, 216, 230);
        } else if (type == BEACH) {
            g->set_color(238, 214, 175);
        } else if (type == ISLAND) {
            g->set_color(205, 183, 158);
        } else if (type == GOLFCOURSE) {
            g->set_color(119, 221, 119);
        } else if (type == BUILDING) {
            g->set_color(169, 169, 169);
        } else if (type == GLACIER) {
            g->set_color(230, 240, 240);
        } else {
            g->set_color(0, 0, 0);
        }

        // Draw feature shape
        if (type != RIVER && type != STREAM) {
            g->fill_poly(points);
        } else {
            g->set_line_width(2);
            for (size_t j = 0; j < points.size() - 1; j++) {
                g->draw_line(points[j], points[j + 1]);
            }
        }
    }
}



void drawRoads(ezgl::renderer *g, double zoomLevel) {
    for (int i = 0; i < roads.size(); i++) {
        int roadType = road_types[i];

        // Skip roads based on zoom level
        if (zoomLevel < 2 && roadType != 3 ) continue;  // Show only highways
        if (zoomLevel < 4 && ((roadType == 1)||(roadType == 0))) continue;  // Hide secondary roads
        if (zoomLevel < 30 && roadType == 0) continue;  // Hide main roads
        
        // Set road color & width
        if (roadType == 3) {
            g->set_color(255, 140, 0);  // Highways
            g->set_line_width(3);
        } else if (roadType == 2) {
            g->set_color(150, 150, 150);  // Major roads
            g->set_line_width(3);
        } else {
            g->set_color(ezgl::WHITE);  // Secondary roads
            g->set_line_width(1);
        }

        // Draw road as a polyline
        for (size_t j = 0; j < roads[i].size() - 1; j++) {
            g->draw_line(roads[i][j], roads[i][j + 1]);
        }
    }
}

void drawPOIs(ezgl::renderer *g, double zoomLevel){
    poi_icon = g->load_png("libstreetmap/resources/point_of_interest.png");
    int scalingFac = 2000000/g->get_visible_world().area();
    int iconFac = std::min(scalingFac, 5);
    for (size_t i=0; i<POIs.size();i++){
        if (g->get_visible_world().area() < 2000000 && g->get_visible_world().contains(POIs[i])){
            g->draw_surface(poi_icon,POIs[i], 0.01*iconFac);
            if (scalingFac>=50){
                g->set_color(0,0,0);
                g->set_font_size(fontSize);
                ezgl::point2d textPos (POIs[i].x, POIs[i].y-3500/zoomLevel);
                g->draw_text(textPos, poiNames[i], 5, 5);
            }
        }
    }
    g->free_surface(poi_icon);
}

double getZoomLevel(ezgl::renderer *g, double initial_width) {
    double current_width = g->get_visible_world().width();
    return initial_width / current_width;  // Zoom ratio
}

// Function to Draw River Names Along the River's Path
void drawRiverNames(ezgl::renderer *g, double zoomLevel) {
    if (zoomLevel < 99) return;  // Skip if zoom level is too low

    for (FeatureIdx i = 0; i < getNumFeatures(); i++) {
        FeatureType type = getFeatureType(i);
        if (type != RIVER && type != STREAM) continue;  // Only process rivers and streams

        std::string featureName = getFeatureName(i);
        if (featureName.empty() || featureName == "<noname>") continue;  // Skip unnamed rivers

        for (int j = 0; j < getNumFeaturePoints(i) - 1; j++) {
            LatLon p1 = getFeaturePoint(i, j);
            LatLon p2 = getFeaturePoint(i, j + 1);

            if (((p1.longitude()-p2.longitude())*(p1.longitude()-p2.longitude()) +(p1.latitude()-p2.latitude())*(p1.latitude()-p2.latitude()))<36)
            {
                continue;
            }
            

            ezgl::point2d point1 = {x_from_lon(p1.longitude()), y_from_lat(p1.latitude())};
            ezgl::point2d point2 = {x_from_lon(p2.longitude()), y_from_lat(p2.latitude())};
            

            // Calculate midpoint
            ezgl::point2d mid((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);

            // Compute angle using m1 function
            double angle = atan2(point2.y - point1.y, point2.x - point1.x) * 180.0 / M_PI; // Ensure you use the correct function
            if (angle < 0) angle += 180;  // Keep text upright

            // Draw the river name at the midpoint
            g->set_color(ezgl::BLACK);
            g->set_font_size(fontSize);
            g->set_text_rotation(angle);
            g->draw_text(mid, featureName);
        }
    }
}


void drawFeatureNames(ezgl::renderer *g, double zoomLevel) {
    if (zoomLevel < 165) return;

    for (FeatureIdx i = 0; i < getNumFeatures(); i++) {
        FeatureType type = getFeatureType(i);
        if (type == RIVER || type == STREAM) continue; 
        std::string featureName = getFeatureName(i);
        if (featureName.empty() || featureName == "<noname>") continue;

        double featureArea = findFeatureArea(i);
        if (featureArea < (100*zoomLevel/300)) continue;  

        ezgl::point2d center = findLargestInscribedRectangle(i);
        g->set_color(ezgl::BLACK);

    

        g->set_font_size(fontSize);
        g->draw_text(center, featureName);
    }
}



// Finds the largest inscribed rectangle inside a closed feature (this part is helped by chatgpt)
ezgl::point2d findLargestInscribedRectangle(FeatureIdx feature_id) {
    int numPoints = getNumFeaturePoints(feature_id);
    if (numPoints < 3) return {0, 0};  // Invalid shape

    double min_x = DBL_MAX, min_y = DBL_MAX;
    double max_x = -DBL_MAX, max_y = -DBL_MAX;

    std::vector<ezgl::point2d> points;
    for (int i = 0; i < numPoints; i++) {
        LatLon latlon = getFeaturePoint(feature_id, i);
        ezgl::point2d p = {x_from_lon(latlon.longitude()), y_from_lat(latlon.latitude())};
        points.push_back(p);
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    // Compute the rectangle center
    ezgl::point2d center((min_x + max_x) / 2, (min_y + max_y) / 2);
    return center;
}



void pre_load_road_data() {
    highways.clear();
    main_roads.clear();
    secondary.clear();
    minor.clear();

    for (int i = 0; i < getNumStreetSegments(); i++) {
        StreetSegmentInfo seg = getStreetSegmentInfo(i);
        std::string streetName = getStreetName(seg.streetID);
        if (streetName.empty() || streetName == "<unkown>") continue;
        RoadSegment road;
        road.length = findStreetSegmentLength(i);
        road.roadType = classify_road(seg.wayOSMID);
        road.name = getStreetName(seg.streetID);

        // **Skip Short Segments Based on Road Type**
        if (road.roadType == 3 && road.length < 300) continue;
        if (road.roadType == 2 && road.length < 150) continue;
        if (road.roadType == 1 && road.length < 80) continue;
        if (road.roadType == 0 && road.length < 50) continue;

        // **Convert Start & End**
        ezgl::point2d start = {
            x_from_lon(getIntersectionPosition(seg.from).longitude()), 
            y_from_lat(getIntersectionPosition(seg.from).latitude())
        };

        ezgl::point2d end = {
            x_from_lon(getIntersectionPosition(seg.to).longitude()), 
            y_from_lat(getIntersectionPosition(seg.to).latitude())
        };

        // **Find Midpoint Directly**
        ezgl::point2d bestMid = start;
        double minDiff = DBL_MAX;

        for (int j = 0; j < seg.numCurvePoints; j++) {
            LatLon curve = getStreetSegmentCurvePoint(i, j);
            ezgl::point2d curvePoint = {x_from_lon(curve.longitude()), y_from_lat(curve.latitude())};

            double diff = fabs(curvePoint.x - (start.x + end.x) / 2) + fabs(curvePoint.y - (start.y + end.y) / 2);
            if (diff < minDiff) {
                minDiff = diff;
                bestMid = curvePoint;
            }
        }

        road.midpoint = bestMid;
        road.angle = atan2(end.y - start.y, end.x - start.x) * 180.0 / M_PI;
        if (road.angle < 0) road.angle += 180; // Keep text upright

        // **Sort into Vectors**
        if (road.roadType == 3) highways.push_back(road);
        else if (road.roadType == 2) main_roads.push_back(road);
        else if (road.roadType == 1) secondary.push_back(road);
        else minor.push_back(road);
    }
}