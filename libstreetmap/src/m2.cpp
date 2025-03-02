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
void drawFeatures(ezgl::renderer *g, double zoomLevel);
void drawRoads(ezgl::renderer *g, double zoomLevel);
double x_from_lon(double lon);
double y_from_lat(double lat);
void loadHighway();
int classify_road(OSMID way_id);
void calculate_map_bound(double &min_lat, double &max_lat, double &min_lon, double &max_lon);
//void drawStreetSegments(ezgl::renderer *g, int priority);
void drawStreetNames(ezgl::renderer *g, double zoomLevel);
void drawOneWayArrows(ezgl::renderer *g);
double getZoomLevel(ezgl::renderer *g, double initial_width);
// for showing intersections
std::string getInput(GtkSearchEntry *entry);
void button_clicked(GtkWidget *widget, gpointer data);
void drawIntersect(ezgl::renderer *g);
void setUpShow(ezgl::application *app, bool /*unused*/);
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets2(std::pair<StreetIdx, StreetIdx> street_ids);
void autoComplete(GtkSearchEntry *entry);
void dropMenu(ezgl::application *app);

void load_road_data();
void load_poi_data();
void draw_main_canvas(ezgl::renderer *g);
void setInterface(ezgl::application &application);
void load_street_names();


// global variables
std::vector<std::vector<ezgl::point2d>> roads;  // Store each road as a list of points
//<a href="https://www.flaticon.com/free-icons/poi" title="poi icons">Poi icons created by Muhammad_Usman - Flaticon</a>
ezgl::surface *poi_icon;

std::vector<ezgl::point2d> POIs;
std::vector<int> road_types;
std::unordered_map<OSMID, std::string> osmHighway;

// for showing two input intersections
std::vector<IntersectionIdx> showIntersection;
GtkListStore *street_list_store;

double avgLat = 0.0;

// Convert Latitude/Longitude to X/Y using Equirectangular Projection
double x_from_lon(double lon) {
    return lon * kDegreeToRadian * kEarthRadiusInMeters * cos(avgLat * kDegreeToRadian);
}

double y_from_lat(double lat) {
    return lat * kDegreeToRadian * kEarthRadiusInMeters;
}

//load only highway information
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
   else if(roadType == "primary" || roadType == "tertiary" ){
      return 2;
   }
   else if(roadType == "secondary"){
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

//one way won't work
void drawOneWayArrows(ezgl::renderer *g) {
    double zoomLevel = getZoomLevel(g, g->get_visible_world().width());

    // Only show arrows when zoom level is sufficiently high
    double minZoomForArrows = 8.0;
    if (zoomLevel < minZoomForArrows) {
        return;
    }

    double arrowSize = std::max(6.0, zoomLevel * 1.5);  // Scale arrows with zoom level

    for (int segmentID = 0; segmentID < getNumStreetSegments(); segmentID++) {
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segmentID);
        if (!segmentInfo.oneWay) continue;

        LatLon pointA = getIntersectionPosition(segmentInfo.from);
        LatLon pointB = getIntersectionPosition(segmentInfo.to);

        ezgl::point2d start(x_from_lon(pointA.longitude()), y_from_lat(pointA.latitude()));
        ezgl::point2d end(x_from_lon(pointB.longitude()), y_from_lat(pointB.latitude()));

        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double length = sqrt(dx * dx + dy * dy);

        // Ensure that at least one arrow appears
        double arrowSpacing = std::max(20.0, length / 3.0);
        if (length < arrowSpacing) {
            continue;  // Skip if road is too short for arrows
        }

        double ux = dx / length;
        double uy = dy / length;

        // Draw multiple arrows along the road
        for (double j = arrowSpacing / 2; j < length; j += arrowSpacing) {
            ezgl::point2d mid(start.x + j * ux, start.y + j * uy);
            ezgl::point2d arrowLeft(mid.x - arrowSize * uy, mid.y + arrowSize * ux);
            ezgl::point2d arrowRight(mid.x + arrowSize * uy, mid.y - arrowSize * ux);
            ezgl::point2d arrowTip(mid.x + arrowSize * ux * 2, mid.y + arrowSize * uy * 2);

            g->set_color(ezgl::BLACK);
            g->set_line_width(2);
            g->draw_line(mid, arrowTip);  // Main arrow line
            g->draw_line(arrowTip, arrowLeft);
            g->draw_line(arrowTip, arrowRight);
        }
    }
}

//functions to implement search bar and find button
std::string getInput(GtkSearchEntry *entry){
    return std::string(gtk_entry_get_text(GTK_ENTRY(entry)));
}

// std::vector<IntersectionIdx> findIntersectionsOfTwoStreets2(std::pair<StreetIdx, StreetIdx> street_ids) {
//     // Get all intersections of both streets using available m1 functions
//     std::vector<IntersectionIdx> street1Intersections = findIntersectionsOfStreet(street_ids.first);
//     std::vector<IntersectionIdx> street2Intersections = findIntersectionsOfStreet(street_ids.second);

//     std::vector<IntersectionIdx> commonIntersections;

//     // Use an unordered_set for quick lookups of street2 intersections
//     std::unordered_set<IntersectionIdx> street2Set(street2Intersections.begin(), street2Intersections.end());

//     // Find intersections that exist in both streets
//     for (IntersectionIdx intersection : street1Intersections) {
//         if (street2Set.find(intersection) != street2Set.end()) {
//             commonIntersections.push_back(intersection);
//         }
//     }

//     // Remove duplicates (if necessary)
//     std::sort(commonIntersections.begin(), commonIntersections.end());
//     commonIntersections.erase(std::unique(commonIntersections.begin(), commonIntersections.end()), commonIntersections.end());

//     return commonIntersections;
// }


void button_clicked(GtkWidget *widget, gpointer data){
    if (data == nullptr) {
    std::cerr << "Error: app is null in findButton" << std::endl;
    return;
    }

    ezgl::application *app = static_cast<ezgl::application*>(data);
    std::cout << "Find button clicked" << std::endl;
    std::string street1 = getInput(GTK_SEARCH_ENTRY(app->get_object("street_1")));
    std::string street2 = getInput(GTK_SEARCH_ENTRY(app->get_object("street_2")));
    //convert name to street id
    std::vector<StreetIdx> street1ID = findStreetIdsFromPartialStreetName(street1);
    std::vector<StreetIdx> street2ID = findStreetIdsFromPartialStreetName(street2);
    std::cout << "Street 1: " << street1 << " -> IDs found: " << street1ID.size() << std::endl;
    std::cout << "Street 2: " << street2 << " -> IDs found: " << street2ID.size() << std::endl;
    if(street1ID.empty() || street2ID.empty()){
        std::cout << "Can't find street name for your typed input" << std::endl;
        app->refresh_drawing();
        return;
    }
    
    showIntersection.clear();
    for(StreetIdx s1: street1ID){
        for(StreetIdx s2: street2ID){
            std::vector<IntersectionIdx> temp = findIntersectionsOfTwoStreets({s1, s2});
            showIntersection.insert(showIntersection.end(), temp.begin(), temp.end());
        }
    }

    if(showIntersection.empty()){
        std::cout << "Intersection found" << std::endl;
        std::string intersectionList;
        for (IntersectionIdx id : showIntersection)
        {
            intersectionList += getIntersectionName(id) + "\n";
        }
        GtkWidget *dialog = gtk_message_dialog_new(GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(app->get_object("MainWindow")))),
                                                   GTK_DIALOG_MODAL,
                                                   GTK_MESSAGE_INFO,
                                                   GTK_BUTTONS_CLOSE,
                                                   "Found %lu intersections:\n%s",
                                                   showIntersection.size(),
                                                   intersectionList.c_str());
        gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_destroy(dialog);
    }
    else{
        std::cout << "Intersection found" << std::endl;
        for(IntersectionIdx id: showIntersection){
            std::cout << "Intersection " << id << getIntersectionName(id) << std::endl;
        }
    }
    app->refresh_drawing();
}

void drawIntersect(ezgl::renderer *g){
    g -> set_color(ezgl::BLUE);
    for(IntersectionIdx id: showIntersection){
        LatLon pos = getIntersectionPosition(id);
        ezgl::point2d intersectPos = {x_from_lon(pos.longitude()), y_from_lat(pos.latitude())};

        ezgl::point2d topLeft(intersectPos.x - 10, intersectPos.y- 10);
        ezgl::point2d bottomRight(intersectPos.x + 10, intersectPos.y + 10);
        g->fill_rectangle(topLeft, bottomRight);
    }
}

void setUpShow(ezgl::application *app, bool /*unused*/){
    dropMenu(app);
    GtkSearchEntry *entry1 = GTK_SEARCH_ENTRY(app -> get_object("street_1"));
    GtkSearchEntry *entry2 = GTK_SEARCH_ENTRY(app -> get_object("street_2"));
    GtkWidget *findButton = GTK_WIDGET(app -> get_object("find_button"));
    if(findButton){
        g_signal_connect(findButton, "clicked", G_CALLBACK(button_clicked), app);
    }
    else{
        std::cerr << "Error: findButton not found in UI" << std::endl;
    }

    if(entry1 && entry2){
        autoComplete(entry1);
        autoComplete(entry2);
    }
    else{
        std::cerr << "Error: could not find searched entry widgets" << std::endl;
    }
}

// complete the auto display of related streetnames
// store street names
void load_street_names(){
    street_list_store = gtk_list_store_new(1, G_TYPE_STRING);
    GtkTreeIter iter;
    for(StreetIdx i = 0; i < getNumStreets(); i++){
        std::string street_name = getStreetName(i);
        gtk_list_store_append(street_list_store, &iter);
        gtk_list_store_set(street_list_store, &iter, 0, street_name.c_str(), -1);
    }
}

void autoComplete(GtkSearchEntry *entry){
    GtkEntryCompletion *completion = gtk_entry_completion_new();
    gtk_entry_completion_set_text_column(completion, 0);
    gtk_entry_completion_set_model(completion, GTK_TREE_MODEL(street_list_store));
    gtk_entry_set_completion(GTK_ENTRY(entry), completion);
    g_object_unref(completion);
}

// connect a dropdown menu in search bars
void dropMenu(ezgl::application *app){
    GtkSearchEntry *entry1 = GTK_SEARCH_ENTRY(app->get_object("street_1"));
    GtkSearchEntry *entry2 = GTK_SEARCH_ENTRY(app->get_object("street_2"));
    if(entry1 && entry2){
        autoComplete(entry1);
        autoComplete(entry2);
    }
    else{
        std::cerr << "Error: Could not find search entry widgets" << std::endl;
    }
}

// Load Roads and Convert to ezgl::point2d
void load_road_data() {
    roads.clear();
    road_types.clear();

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
void load_poi_data(){
    POIs.clear();
    for (int i=0; i<getNumPointsOfInterest(); i++){
        LatLon pos = getPOIPosition(i);
        
        ezgl::point2d projection(x_from_lon(pos.longitude()), y_from_lat(pos.latitude()));
        POIs.push_back(projection);
    }
}

// Draw Roads Based on Classification
void draw_main_canvas(ezgl::renderer *g)
{
   g->set_color(220, 220, 220);
   g->fill_rectangle(g->get_visible_world());
   
   static double initial_width = g->get_visible_world().width();
   double zoomLevel = getZoomLevel(g, initial_width);

    drawFeatures(g, zoomLevel);
    drawRoads(g, zoomLevel);
    drawOneWayArrows(g);

    if(!showIntersection.empty()){
        drawIntersect(g);
    }

    poi_icon = g->load_png("libstreetmap/resources/point_of_interest.png");
    int scalingFac = 2000000/g->get_visible_world().area();
    scalingFac = std::min(scalingFac, 5);
    for (size_t i=0; i<POIs.size();i++){
        if (g->get_visible_world().area() < 2000000 && g->get_visible_world().contains(POIs[i])){
            g->draw_surface(poi_icon,POIs[i], 0.03*scalingFac);
        }
    }
    g->free_surface(poi_icon);


}

// Set Initial View Using LatLon Bounds
void setInterface(ezgl::application &application) {
    load_street_names();
    dropMenu(&application);
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
    std::cout << "Loading UI from: " << settings.main_ui_resource << std::endl;
    ezgl::application application(settings);
    setInterface(application);
    loadHighway();
    load_road_data();   
    load_poi_data();
    application.run(setUpShow, nullptr, nullptr, nullptr);
}

void drawFeatures(ezgl::renderer *g, double zoomLevel) {
    for (FeatureIdx i = 0; i < getNumFeatures(); i++) {
        FeatureType type = getFeatureType(i);
        int numPoints = getNumFeaturePoints(i);
        if (numPoints < 2) continue;

        // Skip drawing buildings unless zoom > 30
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
            g->set_line_width(3);
        }

        // Draw road as a polyline
        for (size_t j = 0; j < roads[i].size() - 1; j++) {
            g->draw_line(roads[i][j], roads[i][j + 1]);
        }
    }
}

double getZoomLevel(ezgl::renderer *g, double initial_width) {
    double current_width = g->get_visible_world().width();
    return initial_width / current_width;  // Zoom ratio
}