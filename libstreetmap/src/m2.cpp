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
extern std::vector<std::vector<StreetSegmentIdx>> streetSegmentVector; 

// function declarations
void drawFeatures(ezgl::renderer *g, double zoomLevel);
void drawRoads(ezgl::renderer *g, double zoomLevel);
void drawPOIs(ezgl::renderer *g, double zoomLevel);
void drawIntersectionHighlight(ezgl::renderer *g);

double x_from_lon(double lon);
double y_from_lat(double lat);
double lon_from_x(double x);
double lat_from_y(double y);
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

struct Intersection{
    LatLon pos;
    std::string name;
    bool highlight;
};


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
std::vector<Intersection> intersections;
std::unordered_map<OSMID, std::string> osmHighway;




double zoomLevel;
double avgLat;
double fontSize;

// Define the structure *before* using it in load_road_data()
struct RoadLabel {
    ezgl::point2d position;  // Label position (precomputed for rendering)
    double angle;            // Rotation angle for correct text alignment
    std::string name;        // Street name
    int roadType;            // Road classification (3 = highways, 2 = main roads, etc.)
};


std::vector<RoadLabel> highways;   // RoadType 3
std::vector<RoadLabel> main_roads; // RoadType 2
std::vector<RoadLabel> secondary;  // RoadType 1
std::vector<RoadLabel> minor;      // RoadType 0







double x_from_lon(double lon) {
    return lon * kDegreeToRadian * kEarthRadiusInMeters * cos(avgLat * kDegreeToRadian);
}

double y_from_lat(double lat) {
    return lat * kDegreeToRadian * kEarthRadiusInMeters;
}

double lon_from_x(double x){
    return x / kDegreeToRadian / kEarthRadiusInMeters / cos(avgLat * kDegreeToRadian);
}

double lat_from_y(double y){
    return y / kDegreeToRadian / kEarthRadiusInMeters;
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



void loadIntersections(){
    for(int i=0; i<getNumIntersections(); i++){
        Intersection newInter;
        newInter.name = getIntersectionName(i);
        newInter.pos = getIntersectionPosition(i);
        newInter.highlight = false;
        intersections.push_back(newInter);
    }
}



// Draw Roads Based on Classification
void draw_main_canvas(ezgl::renderer *g)
{
   g->set_color(220, 220, 220);
   g->fill_rectangle(g->get_visible_world());
   
   static double initial_width = g->get_visible_world().width();
    zoomLevel = getZoomLevel(g, initial_width);
   
   fontSize = std::max(6.0, 9.0 * zoomLevel / 300.0);

    drawFeatures(g, zoomLevel);
    drawRoads(g, zoomLevel);
    if (zoomLevel > 260)
    {
        drawPOIs(g, zoomLevel);
    }
    drawStreetNames(g,zoomLevel);
    drawIntersectionHighlight(g); 
   
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

void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y){
    LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));
    int inter_id = findClosestIntersection(pos);
    if(findDistanceBetweenTwoPoints(pos, getIntersectionPosition(inter_id)) < 500/zoomLevel){
        if (!intersections[inter_id].highlight){
            intersections[inter_id].highlight = true;
            std::stringstream ss;
            ss << "Intersection: "<<intersections[inter_id].name;
            app->update_message(ss.str());
        }
        else{
            intersections[inter_id].highlight = false;
        }
        app->refresh_drawing();
    }
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
    loadIntersections();
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

void drawIntersectionHighlight(ezgl::renderer *g){
    for(int i=0; i<intersections.size(); i++){
        if(intersections[i].highlight && zoomLevel > 5){
            g->set_color(255,0,0);
            g->fill_arc(ezgl::point2d(x_from_lon(intersections[i].pos.longitude()), y_from_lat(intersections[i].pos.latitude())), 1/50*zoomLevel, 0, 360);
        }
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



void drawStreetNames(ezgl::renderer *g, double zoomLevel) {
    if (zoomLevel < 100) return;  // Skip rendering at low zoom levels

    g->set_font_size(fontSize);
    g->set_color(ezgl::BLACK);

    // **Always draw highways if zoom level is at least 100**
    if (zoomLevel >= 166) {
        for (const RoadLabel &road : highways) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }

    // **Draw main roads if zoom level is at least 166**
    if (zoomLevel >= 166) {
        for (const RoadLabel &road : main_roads) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }

    // **Draw secondary roads if zoom level is at least 25000**
    if (zoomLevel >= 25000) {
        for (const RoadLabel &road : secondary) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }

    // **Draw minor roads if zoom level is at least 5000**
    if (zoomLevel >= 5000) {
        for (const RoadLabel &road : minor) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }
}



void pre_load_road_data() {
    highways.clear();
    main_roads.clear();
    secondary.clear();
    minor.clear();

    for (StreetIdx street_id = 0; street_id < getNumStreets(); ++street_id) {
        const auto& segment_ids = streetSegmentVector[street_id];
        if (segment_ids.empty()) continue;

        // **Step 1: Compute Total Street Length**
        double total_length = 0.0;
        for (StreetSegmentIdx seg_id : segment_ids) {
            total_length += findStreetSegmentLength(seg_id);
        }
        if (total_length < 500) continue;  // Ignore short streets

        // **Step 2: Generate Labels Every 500m**
        double accumulated_distance = 0.0;
        ezgl::point2d last_label_position;

        for (StreetSegmentIdx seg_id : segment_ids) {
            StreetSegmentInfo seg_info = getStreetSegmentInfo(seg_id);

            // **Convert LatLon to pixel coordinates**
            ezgl::point2d start = {
                x_from_lon(getIntersectionPosition(seg_info.from).longitude()),
                y_from_lat(getIntersectionPosition(seg_info.from).latitude())
            };

            ezgl::point2d end = {
                x_from_lon(getIntersectionPosition(seg_info.to).longitude()),
                y_from_lat(getIntersectionPosition(seg_info.to).latitude())
            };

            // **Step 3: Label Placement Along the Street**
            double segment_length = findStreetSegmentLength(seg_id);
            accumulated_distance += segment_length;

            if (accumulated_distance >= 500.0) {
                // Compute label position (midpoint of the segment)
                ezgl::point2d label_pos = {(start.x + end.x) / 2, (start.y + end.y) / 2};

                // Compute rotation angle
                double dx = end.x - start.x;
                double dy = end.y - start.y;
                double angle = atan2(dy, dx) * 180.0 / M_PI;
                if (angle < 0) angle += 180;  // Normalize angle

                // Store label
                RoadLabel label = {label_pos, angle, getStreetName(street_id)};
                
                // **Step 4: Assign to Correct Road Type**
                int road_class = classify_road(seg_info.wayOSMID);
                switch (road_class) {
                    case 3: highways.push_back(label); break;
                    case 2: main_roads.push_back(label); break;
                    case 1: secondary.push_back(label); break;
                    default: minor.push_back(label); break;
                }

                accumulated_distance = 0;  // Reset distance counter
            }
        }
    }
}
