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
#include "m3.h"
#include "OSMDatabaseAPI.h"
#include <ezgl/application.hpp>
#include <ezgl/graphics.hpp>
#include <ezgl/rectangle.hpp>
#include <iostream>
#include <unordered_map>
#include <sstream>  // Required for std::istringstream
#include <chrono>
#include <fstream> 
#include <unordered_set>
#include <set>
extern std::vector<std::vector<StreetSegmentIdx>> streetSegmentVector; 

// helper function declarations
void drawFeatures(ezgl::renderer *g);
void drawRoads(ezgl::renderer *g);
void drawPOIs(ezgl::renderer *g);
void drawIntersectionHighlight(ezgl::renderer *g);
void drawOneWayArrows(ezgl::renderer *g);
void drawPath(ezgl::renderer *g);

double x_from_lon(double lon);
double y_from_lat(double lat);
double lon_from_x(double x);
double lat_from_y(double y);
void loadHighway();
void loadPOIs();
int classify_road(OSMID way_id);
void calculate_map_bound(double &min_lat, double &max_lat, double &min_lon, double &max_lon);
void drawStreetNames(ezgl::renderer *g);
double getZoomLevel(ezgl::renderer *g, double initial_width);
std::string getInput(GtkSearchEntry *entry);
void button_clicked(GtkWidget *, gpointer data);
void drawIntersect(ezgl::renderer *g);
void setUpShow(ezgl::application *app, bool /*unused*/);
void autoComplete(GtkSearchEntry *entry, ezgl::application *app);
void dropMenu(ezgl::application *app);
void loadIntersections();
void switchMap(GtkComboBoxText* self, ezgl::application* app);
void nightMode(ezgl::application *app, bool /*Window*/);
gboolean night_switch(GtkSwitch *, gboolean switch_state, ezgl::application *app);
void StreetSelect(GtkComboBox *self, gpointer data);
gboolean on_match_selected(GtkEntryCompletion *completion, GtkTreeModel *model, GtkTreeIter *iter, gpointer data);
std::vector<std::string> getIntersectStreets(const std::string &streetName);
void cleanHighlight(GtkSearchEntry *entry, gpointer data);
void onTyped(GtkEditable *editable, gpointer data);
std::string trim(const std::string &s);
void buildIntersectionStreetSegmentMap();
void drawNavigatePOI();

ezgl::point2d findLargestInscribedRectangle(FeatureIdx feature_id);
std::string splitTextIntoLines(const std::string& text);
void drawFeatureShapes(ezgl::renderer *g);
void drawFeatureNames(ezgl::renderer *g);
void drawRiverNames(ezgl::renderer *g);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* , double x, double y);

// functions to preload & load the street intersection informations
void showStreetNames(GtkWidget *widget, gpointer data);
void showBuildings(GtkWidget * , gpointer data);
void showBuildingnames(GtkWidget * , gpointer data);
void showPOIS(GtkWidget * , gpointer data);
void showDirections(GtkWidget * , gpointer data);
// show textmessage window for help and tutorials
void showHelp(GtkWidget *, gpointer data);
void closeHelpWindow(GtkWidget *, gpointer data);

// draw the scale bar
void drawScale(ezgl::renderer *g);

struct Intersection{
    LatLon pos;
    std::string name;
    bool highlight;
};

void pre_load_road_data();

void load_road_data();
void load_poi_data();
void draw_main_canvas(ezgl::renderer *g);
void setInterface(ezgl::application &application);
void load_street_names();
void preloadFeatureDrawingOrder();
void drawPathArrows(ezgl::renderer *g);
void printTurnMessageIfAny(ezgl::application* app);
bool checkPath(ezgl::application* app);


// Stores road data as a vector of point sequences (each road is represented as a series of points)
std::vector<std::vector<ezgl::point2d>> roads;  

// Surface for displaying POI icons
ezgl::surface *poi_icon; 
ezgl::surface *rest_icon;
ezgl::surface *hosp_icon;
ezgl::surface *bike_icon;
ezgl::surface *atm_icon;

// Stores projected (x, y) coordinates of Points of Interest (POIs)
std::vector<ezgl::point2d> POIs;

// Stores road classification types (highways, main roads, secondary roads, minor roads)
std::vector<int> road_types;

// Stores intersections found when searching for common points between two streets
std::vector<IntersectionIdx> showIntersection;

// GTK ListStore for storing street names to be used in the search bar autocomplete feature
GtkListStore *street_list_store;

// Stores whether each road segment is one-way (true = one-way, false = two-way)
std::vector<bool> oneWayRoad;

// Stores names of POIs, indexed similarly to the POIs vector
std::vector<std::string> poiNames;

// Stores all intersections in the map, including their name, location, and highlight status
std::vector<Intersection> intersections;

//Store all map names
std::vector<std::string> mapOptions = {"Select Map:", "Beijing, China","Beirut, Lebanon", "Berlin, Germany", "Boston, USA", 
    "Cape Town, South Africa", "Golden Horseshoe, Canada", "Hamilton, Canada", "Hong Kong, China",
    "Iceland", "Interlaken, Switzerland", "London, England", "New Delhi, India", "New York, USA", "Rio de Janeiro, Brazil", 
    "Saint Helena", "Singapore", "Tehran, Iran", "Tokyo, Japan", "Toronto, Canada"};

// Maps OpenStreetMap (OSM) Way IDs to their corresponding road type (e.g., highway, residential, etc.)
std::unordered_map<OSMID, std::string> osmHighway;

// store the highlighted street segments
std::vector<StreetSegmentIdx> highlightStreet;

std::vector<std::vector<StreetSegmentIdx>> intersectionStreetSegments;

// Global rendering variables
double zoomLevel;  // Current zoom level of the map
double avgLat;     // Average latitude of the map, used for coordinate conversions
double fontSize;   // Font size for text rendering, adjusted dynamically based on zoom level

// Boolean flags for controlling the display of various map elements
bool showstreetname = false;   // Toggle for displaying street names
bool showBuildingname = true;  // Toggle for displaying building names
bool showBuilding = true;      // Toggle for displaying buildings
bool showPOI = true;           // Toggle for displaying POIs (points of interest)
bool showDirection = true;     // Toggle for displaying one-way road directions 

bool nightmode = false;

// Define the structure *before* using it in load_road_data()
struct RoadLabel {
    ezgl::point2d position;  
    double angle;            
    std::string name;        
};

std::vector<RoadLabel> highways;   
std::vector<RoadLabel> main_roads; 
std::vector<RoadLabel> secondary;  
std::vector<RoadLabel> minor;     

//The path from one point to another
std::vector<StreetSegmentIdx> currentPath;

//The soreted vectors that has the features from the largest area to the least
std::vector<FeatureIdx> sortedFeatureIndices;

IntersectionIdx clickStart = -1;    // The intersection selected as the path start 
IntersectionIdx clickEnd = -1;      // The intersection selected as the path end 

bool readyToFindPath = false;       // Flag indicating if both intersections have been selected for pathfinding
double turn_penalty = 15.0;         // Turn penalty in seconds applied during pathfinding

// Convert Latitude/Longitude to X/Y using Equirectangular Projection
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

/////Unused function
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

//This Function describes drawing for one way direction
void drawOneWayArrows(ezgl::renderer *g) {
    double minZoom = 240.0;
    if(zoomLevel < minZoom || !showDirection){
        return;
    }
    if(nightmode == true){
        g->set_color(ezgl::WHITE);
    }
    else{
        g->set_color(ezgl::BLACK);
    }
    g->set_line_width(2);

    for(size_t segmentId = 0; segmentId < roads.size(); segmentId++){
        if(!oneWayRoad[segmentId]) continue; // if not oneway then skip
        std::vector<ezgl::point2d> roadPt = roads[segmentId];
        int typeDisplay = road_types[segmentId];
        double baseArrowSize = 4.0;
        double maxArrowSize = 8.0;
        double arrowSize = baseArrowSize * zoomLevel * 0.003 ;
        arrowSize = std::min(arrowSize, maxArrowSize);

        // Adjust spacing dynamically
        double baseSpacing = (typeDisplay >= 2) ? 120.0 : 60.0; // Sparse arrows on highways
        double maxSpacing = (typeDisplay >= 2) ? 300.0 : 150.0; // Further spaced on highways
        double arrowSpacing = std::max(baseSpacing, maxSpacing / zoomLevel);

        ezgl::point2d lastArrowPos = {0, 0}; // Track last arrow position to avoid clutter

        for (size_t i = 0; i < roadPt.size() - 1; i++) {
            ezgl::point2d start = roadPt[i];
            ezgl::point2d end = roadPt[i + 1];

            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double length = sqrt(dx * dx + dy * dy);
            if (length < 30) continue; // Skip very short roads

            double ux = dx / length;
            double uy = dy / length;

            // Reduce arrow clutter on highways and dense roads
            if (typeDisplay >= 2 && i % 2 == 0) continue; // Show fewer arrows on highways

            for (double j = length / 3; j < length; j += arrowSpacing) {
                ezgl::point2d mid(start.x + j * ux, start.y + j * uy);

                // Prevent overlapping arrows on dense areas
                double dist = sqrt(pow(mid.x - lastArrowPos.x, 2) + pow(mid.y - lastArrowPos.y, 2));
                if (dist < arrowSpacing * 0.5) continue;  // Skip drawing if too close to last arrow

                lastArrowPos = mid; // Update last position

                ezgl::point2d arrowLeft(mid.x - arrowSize * uy, mid.y + arrowSize * ux);
                ezgl::point2d arrowRight(mid.x + arrowSize * uy, mid.y - arrowSize * ux);
                ezgl::point2d arrowTip(mid.x + arrowSize * ux * 2, mid.y + arrowSize * uy * 2);

                g->draw_line(mid, arrowTip);
                g->draw_line(arrowTip, arrowLeft);
                g->draw_line(arrowTip, arrowRight);
            }
        }
    }
}

//functions to implement search bar and find button
std::string getInput(GtkSearchEntry *entry){
    return std::string(gtk_entry_get_text(GTK_ENTRY(entry)));
}

void button_clicked(GtkWidget *, gpointer data){
    if (data == nullptr) return;
    ezgl::application *app = static_cast<ezgl::application*>(data);

    std::string input1 = getInput(GTK_SEARCH_ENTRY(app->get_object("street_1")));
    std::string input2 = getInput(GTK_SEARCH_ENTRY(app->get_object("street_2")));

    highlightStreet.clear();
    showIntersection.clear();
    // for each single intersection need to use the seperation to identify input
    auto parseIntersection = [](const std::string &input) -> std::vector<IntersectionIdx> {
        size_t amp = input.find('&');
        if (amp == std::string::npos) return {};
        std::string s1 = input.substr(0, amp);
        std::string s2 = input.substr(amp + 1);
        trim(s1); 
        trim(s2);

        std::vector<StreetIdx> ids1 = findStreetIdsFromPartialStreetName(s1);
        std::vector<StreetIdx> ids2 = findStreetIdsFromPartialStreetName(s2);
        std::vector<IntersectionIdx> results;
        for (StreetIdx id1 : ids1) {
            for (StreetIdx id2 : ids2) {
                std::vector<IntersectionIdx> inters = findIntersectionsOfTwoStreets({id1, id2});
                results.insert(results.end(), inters.begin(), inters.end());
            }
        }
        return results;
    };

    auto highlightInput = [](const std::string &input){
        std::vector<StreetIdx> ids = findStreetIdsFromPartialStreetName(input);
        for(StreetIdx id : ids){
            for(StreetSegmentIdx i = 0; i < getNumStreetSegments(); i++){
                if(getStreetSegmentInfo(i).streetID == id){
                    highlightStreet.push_back(i);
                }
            }
        }
    };

    bool input1IsIntersection = input1.find('&') != std::string::npos;
    bool input2IsIntersection = input2.find('&') != std::string::npos;

    std::vector<IntersectionIdx> srcList, dstList;

    if (input1IsIntersection){
        srcList = parseIntersection(input1);
    } 
    else{
        highlightInput(input1);
    }
    if (input2IsIntersection){
        dstList = parseIntersection(input2);
    }
    else {
        highlightInput(input2);
    }
    // If both entries are valid street names, autocenter the intersection
    if(!input1IsIntersection && !input2IsIntersection){
        intersections.clear();
        currentPath.clear();
        std::vector<StreetIdx> streetList1 = findStreetIdsFromPartialStreetName(input1);
        std::vector<StreetIdx> streetList2 = findStreetIdsFromPartialStreetName(input2);
        for(StreetIdx i: streetList1){
            for(StreetIdx j: streetList2){
                std::vector<IntersectionIdx> streetPOI = findIntersectionsOfTwoStreets({i, j});
                if(!streetPOI.empty()){
                    showIntersection = streetPOI;
                    LatLon pos = getIntersectionPosition(showIntersection[0]);
                    ezgl::point2d centerPos(x_from_lon(pos.longitude()), y_from_lat(pos.latitude()));
                    ezgl::rectangle view(centerPos - ezgl::point2d(500, 500), centerPos + ezgl::point2d(500, 500));
                    std::stringstream ss;
                    ss << "Intersection of: " << input1 << " & " << input2 << "\n\n";
                    ss << "Latitude: " << pos.latitude() << "\nLongitude: " << pos.longitude();
                    GtkWidget *dialog = gtk_message_dialog_new(
                    GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(app->get_object("MainWindow")))),
                    GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE,"%s", ss.str().c_str());
                    gtk_window_set_title(GTK_WINDOW(dialog), "Intersection Position");
                    gtk_dialog_run(GTK_DIALOG(dialog));
                    gtk_widget_destroy(dialog);
                    app -> change_canvas_world_coordinates("MainCanvas", view);
                    app->refresh_drawing();
                    return;
                }
            }
        }
        //if nothing found, show error
        if (!checkPath(app))
            {
            highlightStreet.clear();
            currentPath.clear();
            showIntersection.clear();
            return;
            }
    }
    // If both entries are valid intersections, attempt pathfinding
    else if (input1IsIntersection && input2IsIntersection) {

        intersections.clear();
        highlightStreet.clear();

        if (srcList.empty() || dstList.empty()) {
            GtkWidget *dialog = gtk_message_dialog_new(
                GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(app->get_object("MainWindow")))),
                GTK_DIALOG_MODAL, GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE,
                "No Response for Your Streets."
            );
            gtk_window_set_title(GTK_WINDOW(dialog), "Invalid Input");
            gtk_dialog_run(GTK_DIALOG(dialog));
            gtk_widget_destroy(dialog);
            return;
        }

        // Clear previous intersection highlights 
        for (IntersectionIdx id : showIntersection) {
        intersections[id].highlight = false;
        }
        
        showIntersection.clear();


        IntersectionIdx from = srcList[0];
        IntersectionIdx to = dstList[0];

        currentPath = findPathBetweenIntersections(15.0, {from, to});
        printTurnMessageIfAny(app);

        if (currentPath.empty()) {
            GtkWidget *dialog = gtk_message_dialog_new(
                GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(app->get_object("MainWindow")))),
                GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE,
                "No path found between the two intersections."
            );
            gtk_window_set_title(GTK_WINDOW(dialog), "No Path");
            gtk_dialog_run(GTK_DIALOG(dialog));
            gtk_widget_destroy(dialog);
            return;
        }

        // Highlight and save the new intersections
        intersections[from].highlight = true;
        intersections[to].highlight = true;
        showIntersection = {from, to};

        // center the map to the destination
        LatLon center = getIntersectionPosition(to);
        ezgl::point2d centerPt(x_from_lon(center.longitude()), y_from_lat(center.latitude()));
        ezgl::rectangle view(centerPt - ezgl::point2d(500, 500), centerPt + ezgl::point2d(500, 500));
        app->change_canvas_world_coordinates("MainCanvas", view);
    }
    app->refresh_drawing();
}

// function for drawing the intersections of the streets
void drawIntersect(ezgl::renderer *g){
    g -> set_color(255,0,0);
    for(IntersectionIdx id: showIntersection){
        LatLon pos = getIntersectionPosition(id);
        ezgl::point2d intersectPos = {x_from_lon(pos.longitude()), y_from_lat(pos.latitude())};
        g->fill_arc(intersectPos, 500/zoomLevel, 0, 360);
    }
}

// function to create complete setup before detect window interaction
void setUpShow(ezgl::application *app, bool /*unused*/){
    dropMenu(app);
    nightMode(app, nightmode);

    // Get search entries and find button
    GtkSearchEntry *entry1 = GTK_SEARCH_ENTRY(app -> get_object("street_1"));
    GtkSearchEntry *entry2 = GTK_SEARCH_ENTRY(app -> get_object("street_2"));
    GtkWidget *findButton = GTK_WIDGET(app -> get_object("find_button"));
    // highlight selected street:
    GtkComboBox *selectS1 = GTK_COMBO_BOX(app->get_object("street1"));
    GtkComboBox *selectS2 = GTK_COMBO_BOX(app->get_object("street2"));
    if(findButton == nullptr){
        return;
    }
    else{
        g_signal_connect(findButton, "clicked", G_CALLBACK(button_clicked), app);
    }

    if(entry1){
        autoComplete(entry1, app);
        g_signal_connect(entry1, "changed", G_CALLBACK(onTyped), app);
        g_signal_connect(entry1, "changed", G_CALLBACK(cleanHighlight), app);
    }
    if(entry2){
        autoComplete(entry2, app);
        g_signal_connect(entry2, "changed", G_CALLBACK(onTyped), app);
        g_signal_connect(entry2, "changed", G_CALLBACK(cleanHighlight), app);
    }
    if(selectS1){
        g_signal_connect(selectS1, "changed", G_CALLBACK(on_match_selected), app);
    }
    if(selectS2){
        g_signal_connect(selectS2, "changed", G_CALLBACK(on_match_selected), app);
    }

    // Connect toggle buttons
    GtkWidget *streetnameButton = GTK_WIDGET(app -> get_object("showName"));
    g_signal_connect(streetnameButton, "clicked", G_CALLBACK(showStreetNames), app);
    //std::cout << "detected the showstreetname button" << std::endl;

    GtkWidget *showBuildingButton = GTK_WIDGET(app -> get_object("showbuilding"));
    g_signal_connect(streetnameButton, "clicked", G_CALLBACK(showBuildings), app);
    //std::cout << "detected the showstreetname button" << std::endl;

    GtkWidget *showBuildingnameButton = GTK_WIDGET(app -> get_object("showbuildingname"));
    g_signal_connect(streetnameButton, "clicked", G_CALLBACK(showBuildingnames), app);
    //std::cout << "detected the showstreetname button" << std::endl;

    GtkWidget *showDirectionButton = GTK_WIDGET(app -> get_object("showdirection"));
    g_signal_connect(streetnameButton, "clicked", G_CALLBACK(showDirections), app);
    //std::cout << "detected the showstreetname button" << std::endl;

    GtkWidget *showPOIButton = GTK_WIDGET(app -> get_object("showPOI"));
    g_signal_connect(streetnameButton, "clicked", G_CALLBACK(showPOIS), app);
    //std::cout << "detected the showstreetname button" << std::endl;
    
    GtkWidget *helpButton =  GTK_WIDGET(app -> get_object("HelpButton"));
    g_signal_connect(helpButton, "clicked", G_CALLBACK(showHelp), app);

    GtkWidget *closeHelp = GTK_WIDGET(app -> get_object("CloseHelpWindow"));
    g_signal_connect(closeHelp, "clicked", G_CALLBACK(closeHelpWindow), app);
    app->create_combo_box_text("Select Map", 0, switchMap, mapOptions);
}

//Allows for switching of maps
void switchMap(GtkComboBoxText* self, ezgl::application* app){
    std::string mapAddress = "/cad2/ece297s/public/maps/";
    std::string cityName = "";
    int option = gtk_combo_box_get_active(GTK_COMBO_BOX(self));
    
    //Get the city name
    if(option > 0){
        for(int i=0; i<mapOptions[option].length(); i++){
            if(mapOptions[option][i] == ' ' && mapOptions[option][i-1] == ','){
                cityName.append("_");
            }
            else if(mapOptions[option][i] == ' ' && mapOptions[option][i-1] != ','){
                cityName.append("-");
            }
            else if (mapOptions[option][i] != ','){
                cityName.append(1, static_cast<char>(std::tolower(mapOptions[option][i])));
            }
        }
        mapAddress.append(cityName);
        mapAddress.append(".streets.bin");
        closeMap();
        
        //Draw the new map
        bool load_success = loadMap(mapAddress);
        if(!load_success) {
            std::cerr << "Failed to load map '" << mapAddress << "'\n";
            return;
        }
        std::cout << "Successfully loaded map '" << mapAddress << "'\n";
        load_street_names();  
        dropMenu(app);
        showIntersection.clear();
        double min_lat, max_lat, min_lon, max_lon;
        calculate_map_bound(min_lat, max_lat, min_lon, max_lon);

        ezgl::rectangle initial_world(
            {x_from_lon(min_lon), y_from_lat(min_lat)},
            {x_from_lon(max_lon), y_from_lat(max_lat)}
        );
        
        app->change_canvas_world_coordinates("MainCanvas",initial_world);
        ezgl::renderer *g = app->get_renderer();
        g->set_visible_world(initial_world); 
        pre_load_road_data();
        preloadFeatureDrawingOrder();
        loadHighway();
        load_road_data();
        loadPOIs();
        loadIntersections();
        
        double initial_width = g->get_visible_world().width();
        zoomLevel = getZoomLevel(g, initial_width);
        app->refresh_drawing();
    }
}

void nightMode(ezgl::application *app, bool /*Windowapp*/){
    GObject *nightSwitch = app -> get_object("NightMode");
    g_signal_connect(nightSwitch, "state-set", G_CALLBACK(night_switch), app);
}

gboolean night_switch(GtkSwitch *, gboolean switch_state, ezgl::application *app){
    nightmode = switch_state;
    app -> refresh_drawing();
    return false;
}

void showHelp(GtkWidget *, gpointer data){
    ezgl::application *app = static_cast<ezgl::application *>(data);
    GtkWidget *helpWindow = GTK_WIDGET(app->get_object("HelpWindow"));
    if(helpWindow){
        gtk_widget_show_all(helpWindow);
    }
    g_signal_connect(helpWindow, "delete-event", G_CALLBACK(gtk_widget_hide_on_delete), NULL);
}

void closeHelpWindow(GtkWidget *, gpointer data){
    ezgl::application *app = static_cast<ezgl::application *>(data);
    GtkWidget *helpWindow = GTK_WIDGET(app->get_object("HelpWindow"));
    if(helpWindow){
        gtk_widget_hide(helpWindow);
    }
}

// store street names
// Load all street names into the global list store
void load_street_names() {
    street_list_store = gtk_list_store_new(1, G_TYPE_STRING);
    GtkTreeIter iter;
    for (StreetIdx i = 0; i < getNumStreets(); i++) {
        std::string street_name = getStreetName(i);
        gtk_list_store_append(street_list_store, &iter);
        gtk_list_store_set(street_list_store, &iter, 0, street_name.c_str(), -1);
    }
}

void buildIntersectionStreetSegmentMap() {
    int numIntersections = getNumIntersections();
    intersectionStreetSegments.clear();
    intersectionStreetSegments.resize(numIntersections);

    for (StreetSegmentIdx seg = 0; seg < getNumStreetSegments(); ++seg) {
        StreetSegmentInfo info = getStreetSegmentInfo(seg);
        intersectionStreetSegments[info.from].push_back(seg);
        intersectionStreetSegments[info.to].push_back(seg);
    }
}

// Attach autocomplete to a GtkSearchEntry
void autoComplete(GtkSearchEntry *entry, ezgl::application *app){
    GtkEntryCompletion *completion = gtk_entry_completion_new();
    gtk_entry_completion_set_text_column(completion, 0);
    gtk_entry_completion_set_model(completion, GTK_TREE_MODEL(street_list_store)); 
    gtk_entry_set_completion(GTK_ENTRY(entry), completion);
    
    // Detect when the user selects an entry
    g_signal_connect(completion, "match-selected", G_CALLBACK(on_match_selected), app);
}

// Highlight selected street from dropdown
void StreetSelect(GtkComboBox *self, gpointer data) {
    ezgl::application *app = static_cast<ezgl::application *>(data);
    const char* streetSelect = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(self));
    if (streetSelect == nullptr) return;

    std::string selection(streetSelect);
    g_free((void*)streetSelect);
    if (selection.empty()) return;

    highlightStreet.clear();
    std::vector<StreetIdx> streetId = findStreetIdsFromPartialStreetName(selection);
    for (StreetIdx s : streetId) {
        for (StreetSegmentIdx i = 0; i < getNumStreetSegments(); i++) {
            StreetSegmentInfo segInfo = getStreetSegmentInfo(i);
            if (segInfo.streetID == s) {
                highlightStreet.push_back(i);
            }
        }
    }
    app->refresh_drawing();
}

gboolean on_match_selected(GtkEntryCompletion *completion, GtkTreeModel *model, GtkTreeIter *iter, gpointer data) {
    gchar *street_name;
    gtk_tree_model_get(model, iter, 0, &street_name, -1);

    if (street_name == nullptr) return FALSE;

    ezgl::application *app = static_cast<ezgl::application *>(data);

    highlightStreet.clear();
    std::vector<StreetIdx> streetId = findStreetIdsFromPartialStreetName(street_name);
    for (StreetIdx s : streetId) {
        for (StreetSegmentIdx i = 0; i < getNumStreetSegments(); i++) {
            StreetSegmentInfo segInfo = getStreetSegmentInfo(i);
            if (segInfo.streetID == s) {
                highlightStreet.push_back(i);
            }
        }
    }

    g_free(street_name);
    app->refresh_drawing();
    return FALSE;  // Allow default handler to run
}

// clearn the highlighted street when recall the search input
void cleanHighlight(GtkSearchEntry *entry, gpointer data){
    ezgl::application *app = static_cast<ezgl::application *>(data);
    const char *input_ptr = gtk_entry_get_text(GTK_ENTRY(entry));
    if(input_ptr == nullptr || strlen(input_ptr) == 0){
        highlightStreet.clear();
        app->refresh_drawing();
    }
}

void onTyped(GtkEditable *editable, gpointer data){
    ezgl::application *app = static_cast<ezgl::application *>(data);
    GtkEntry *entry = GTK_ENTRY(editable);

    std::string input = gtk_entry_get_text(entry);
    GtkEntryCompletion *completion = gtk_entry_get_completion(entry);

    size_t andSign = input.find('&');
    GtkListStore *store = gtk_list_store_new(1, G_TYPE_STRING);
    GtkTreeIter iter;

    if (andSign != std::string::npos) {
        std::string street1 = trim(input.substr(0, andSign));
        auto street1_ids = findStreetIdsFromPartialStreetName(street1);
        // need to parse through all the related intersection street segments give first street name
        std::set<std::string> suggestions;
        for (StreetIdx s1: street1_ids) {
            auto intersecting = getIntersectStreets(getStreetName(s1));
            for (const std::string &s : intersecting) {
                suggestions.insert(street1 + " & " + s);  
            }
        }

        // Autodisplay all the suggestions from the dropdown menu
        for (const std::string &s : suggestions) {
            gtk_list_store_append(store, &iter);
            gtk_list_store_set(store, &iter, 0, s.c_str(), -1);
        }

        gtk_entry_completion_set_model(completion, GTK_TREE_MODEL(store));
        gtk_entry_completion_set_text_column(completion, 0);
        gtk_entry_completion_set_inline_completion(completion, TRUE);
        gtk_entry_completion_set_popup_completion(completion, TRUE);
        gtk_entry_completion_set_inline_selection(completion, TRUE);
        gtk_entry_set_completion(GTK_ENTRY(entry), completion);
        gtk_editable_set_position(GTK_EDITABLE(entry), -1);  // helps force cursor redraw
        gtk_entry_completion_complete(completion); // show dropdown

    } 
    else {
        // User typing normally → full street name suggestions
        for (StreetIdx i = 0; i < getNumStreets(); ++i) {
            std::string name = getStreetName(i);
            if (name.find(input) != std::string::npos) {
                gtk_list_store_append(store, &iter);
                gtk_list_store_set(store, &iter, 0, name.c_str(), -1);
            }
        }

        gtk_entry_completion_set_model(completion, GTK_TREE_MODEL(store));
        gtk_entry_completion_set_text_column(completion, 0);
        gtk_entry_set_completion(GTK_ENTRY(entry), completion);
        gtk_entry_completion_complete(completion);
    }

    if(andSign != std::string::npos){
        std::string s1 = trim(input.substr(0, andSign));
        std::string s2 = trim(input.substr(andSign + 1));
        std::vector<StreetIdx> ids1 = findStreetIdsFromPartialStreetName(s1);
        std::vector<StreetIdx> ids2 = findStreetIdsFromPartialStreetName(s2);
        showIntersection.clear();
        for(StreetIdx i: ids1){
            for(StreetIdx j: ids2){
                auto intersections = findIntersectionsOfTwoStreets({i, j});
                showIntersection.insert(showIntersection.end(), intersections.begin(), intersections.begin());
            }
        }
        app->refresh_drawing();
    }
}

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, last - first + 1);
}

std::vector<std::string> getIntersectStreets(const std::string &streetName) {
    std::vector<std::string> result;
    std::unordered_set<std::string> seen;
    std::vector<StreetIdx> streetIDs = findStreetIdsFromPartialStreetName(streetName);

    for (StreetIdx sid : streetIDs) {
        auto segments = streetSegmentVector[sid];

        for (StreetSegmentIdx segID : segments) {
            StreetSegmentInfo seg = getStreetSegmentInfo(segID);

            for (IntersectionIdx inter : {seg.from, seg.to}) {
                for (StreetSegmentIdx neighborSeg : intersectionStreetSegments[inter]) {
                    StreetSegmentInfo neighborInfo = getStreetSegmentInfo(neighborSeg);
                    std::string neighborName = getStreetName(neighborInfo.streetID);

                    if (neighborName != streetName && seen.insert(neighborName).second) {
                        result.push_back(neighborName);
                    }
                }
            }
        }
    }
    return result;
}

// connect a dropdown menu in search bars
void dropMenu(ezgl::application *app){
    GtkSearchEntry *entry1 = GTK_SEARCH_ENTRY(app->get_object("street_1"));
    GtkSearchEntry *entry2 = GTK_SEARCH_ENTRY(app->get_object("street_2"));
    if(entry1 && entry2){
        autoComplete(entry1, app);
        autoComplete(entry2, app);
    }
}
 
// function for deciding showing streetnames
void showStreetNames(GtkWidget *, gpointer data){
    // if detected, then we change its status
    if(showstreetname == false){
        showstreetname = true;
    }
    else{
        showstreetname = false;
    }
    ezgl::application *app = static_cast<ezgl::application *>(data);
    app -> refresh_drawing();
}

// function for deciding showing building names
void showBuildingnames(GtkWidget * , gpointer data){
    // if detected, then we change its status
    if(showBuildingname == false){
        showBuildingname = true;
    }
    else{
        showBuildingname = false;
    }
    ezgl::application *app = static_cast<ezgl::application *>(data);
    app -> refresh_drawing();
}

// function for deciding showing building 
void showBuildings(GtkWidget * , gpointer data){
    // if detected, then we change its status
    if(showBuilding == false){
        showBuilding = true;
    }
    else{
        showBuilding = false;
    }
    ezgl::application *app = static_cast<ezgl::application *>(data);
    app -> refresh_drawing();
}

// function for deciding showing building POIs
void showPOIS(GtkWidget *, gpointer data){
    // if detected, then we change its status
    if(showPOI == false){
        showPOI = true;
    }
    else{
        showPOI = false;
    }
    ezgl::application *app = static_cast<ezgl::application *>(data);
    app -> refresh_drawing();
}

// function for deciding showing directions
void showDirections(GtkWidget * , gpointer data){
    // if detected, then we change its status
    if(showDirection == false){
        showDirection = true;
    }
    else{
        showDirection = false;
    }
    ezgl::application *app = static_cast<ezgl::application *>(data);
    app -> refresh_drawing();
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
        oneWayRoad.push_back(seg.oneWay);
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

//load the intersections
void loadIntersections(){
    for(int i=0; i<getNumIntersections(); i++){
        Intersection newInter;
        newInter.name = getIntersectionName(i);
        newInter.pos = getIntersectionPosition(i);
        newInter.highlight = false;
        intersections.push_back(newInter);
    }
}

//draw a scale bar
// void drawScale(ezgl::renderer *g) {
//     double screenWidth = g->get_visible_screen().width();
//     double screenHeight = g->get_visible_screen().height();
//     double getWidth = g->get_visible_world().width();
//     double scaleLen = std::max(100.0, std::min(500.0, 100 * (getWidth / 16)));
//     std::string distNum = std::to_string(int(scaleLen)) + "m";

//     double x_start = 50;
//     double y_start = screenHeight - 50;

//     g->set_color(ezgl::BLACK);
//     g->set_line_width(3);
//     g->draw_line({x_start, y_start}, {x_start + scaleLen, y_start});

//     g->set_font_size(30);
//     g->set_color(ezgl::BLACK);
//     g->draw_text({x_start + scaleLen / 2, y_start - 20}, distNum);
// }

// Draw main canvas
void draw_main_canvas(ezgl::renderer *g)
{


    if(nightmode == true){
        g->set_color(17, 20, 24);
    }
    else{
        g->set_color(220, 220, 220);
    }
   g->fill_rectangle(g->get_visible_world());
   
   static double initial_width = g->get_visible_world().width();
   zoomLevel = getZoomLevel(g, initial_width);

   fontSize = std::max(6.0, 9.0 * zoomLevel / 300.0);


    drawFeatures(g);
    drawOneWayArrows(g);

    if(!showIntersection.empty()){
        drawIntersect(g);
    }
    
    if (zoomLevel > 260)
    {
        drawPOIs(g);
    }
    
    drawRoads(g);
    drawIntersectionHighlight(g);
    drawFeatureNames(g);   
    drawRiverNames(g); 
    drawStreetNames(g);
    //drawScale(g);
    drawPath(g);
}

// Set Initial View Using LatLon Bounds
void setInterface(ezgl::application &application) {
    load_street_names();
    dropMenu(&application);
    double min_lat, max_lat, min_lon, max_lon;
    calculate_map_bound(min_lat, max_lat, min_lon, max_lon);
    ezgl::rectangle initial_world({x_from_lon(min_lon), y_from_lat(min_lat)}, {x_from_lon(max_lon), y_from_lat(max_lat)});
    application.add_canvas("MainCanvas", draw_main_canvas, initial_world);
}

// It processes the click to identify and highlight intersections,
// and initiates pathfinding when two valid intersections are selected.
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y) {
    LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));
    int inter_id = findClosestIntersection(pos);
    
    if (findDistanceBetweenTwoPoints(pos, getIntersectionPosition(inter_id)) < 500 / zoomLevel) {
        highlightStreet.clear();
        
        currentPath.clear();

        // Prevent unhighlighting an already selected intersection
        if (clickStart == -1 && clickEnd == -1) {
            // New path start: clear previous highlights
            for (IntersectionIdx id : showIntersection) {
                intersections[id].highlight = false;
            }
            showIntersection.clear();
            

            clickStart = inter_id;
            intersections[inter_id].highlight = true;
            showIntersection.push_back(clickStart);
            app->update_message("🚩 Start: " + getIntersectionName(clickStart));
        } 
        else if (clickStart != -1 && clickEnd == -1) {
            // Prevent double-clicking the same point
            if (inter_id == clickStart) return;

            clickEnd = inter_id;
            intersections[clickEnd].highlight = true;
            
            showIntersection = {clickStart, clickEnd};
            std::stringstream ss;
            ss << "🚩  Start： " << getIntersectionName(clickStart)
               << "  →  📍  End： " << getIntersectionName(clickEnd);
            currentPath = findPathBetweenIntersections(turn_penalty, {clickStart, clickEnd});

             if (!checkPath(app))
            {
            highlightStreet.clear();
            currentPath.clear();
            showIntersection.clear();
            return;
            }

            app->update_message(ss.str());

            clickStart = -1;
            clickEnd = -1;
        }
        printTurnMessageIfAny(app);
        app->refresh_drawing();
    }
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
    pre_load_road_data();
    preloadFeatureDrawingOrder();
    loadHighway();
    load_road_data();   
    //load_poi_data();
    loadPOIs();
    loadIntersections();
    application.run(setUpShow, act_on_mouse_click, nullptr, nullptr);
}

void drawFeatures(ezgl::renderer *g) {
    drawFeatureShapes(g);          
}

//Function for drawing the shapes of the features
void drawFeatureShapes(ezgl::renderer *g) {
    for (FeatureIdx i : sortedFeatureIndices) {
        FeatureType type = getFeatureType(i);
        int numPoints = getNumFeaturePoints(i);
        if (numPoints < 2) continue;

        if (type == BUILDING && (zoomLevel < 60 || !showBuilding))
            continue;

        std::vector<ezgl::point2d> points;
        for (int j = 0; j < numPoints; j++) {
            LatLon latlon = getFeaturePoint(i, j);
            points.push_back(ezgl::point2d(x_from_lon(latlon.longitude()), y_from_lat(latlon.latitude())));
        }

        // Color settings (your existing logic)
        if (nightmode) {
            if (type == PARK || type == GREENSPACE)
                g->set_color(20, 61, 39);
            else if (type == LAKE || type == RIVER || type == STREAM)
                g->set_color(24, 34, 45);
            else if (type == BEACH)
                g->set_color(238, 214, 175);
            else if (type == ISLAND)
                g->set_color(29, 37, 44);
            else if (type == GOLFCOURSE)
                g->set_color(43, 53, 61);
            else if (type == BUILDING)
                g->set_color(80, 80, 80);
            else if (type == GLACIER)
                g->set_color(255, 255, 255);
            else
                g->set_color(30, 30, 30);
        } else {
            if (type == PARK || type == GREENSPACE)
                g->set_color(181, 220, 159);
            else if (type == LAKE || type == RIVER || type == STREAM)
                g->set_color(173, 216, 230);
            else if (type == BEACH)
                g->set_color(238, 214, 175);
            else if (type == ISLAND)
                g->set_color(205, 183, 158);
            else if (type == GOLFCOURSE)
                g->set_color(119, 221, 119);
            else if (type == BUILDING)
                g->set_color(169, 169, 169);
            else if (type == GLACIER)
                g->set_color(255, 255, 255);
            else
                g->set_color(0, 0, 0);
        }

        // Draw
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

//function for drawing roads
void drawRoads(ezgl::renderer *g) {
    for (StreetSegmentIdx segmentId = 0; segmentId < getNumStreetSegments(); segmentId++) {
        StreetSegmentInfo segInfo = getStreetSegmentInfo(segmentId);
        int roadType = classify_road(segInfo.wayOSMID);

        // Skip roads based on zoom level
        if (zoomLevel < 2 && roadType != 3 ) continue;  // Show only highways
        if (zoomLevel < 4 && ((roadType == 1)||(roadType == 0))) continue;  // Hide secondary roads
        if (zoomLevel < 30 && roadType == 0) continue;  // Hide main roads
        
        //show the highlight  street
        bool  streetHighlight = (std::find(highlightStreet.begin(), highlightStreet.end(), segmentId) != highlightStreet.end());

        // Set road color & width
        if(streetHighlight){
            g->set_color(ezgl::BLUE);
            g->set_line_width(3);
        }

        else{ // draw regular roads
            if(nightmode == false){
                if (roadType == 3) {
                    g->set_color(255, 140, 0);  // Highways
                    g->set_line_width(3);
                } else if (roadType == 2) {
                    g->set_color(150, 150, 150);  // Major roads
                    g->set_line_width(2);
                } else {
                    g->set_color(ezgl::WHITE);  // Secondary roads
                    g->set_line_width(1);
                }
            }
            else{ // show nightmode
                if (roadType == 3) {
                    g->set_color(82, 113, 152);  // Highways
                    g->set_line_width(3);
                } else if (roadType == 2) {
                    g->set_color(56, 79, 102);  // Major roads
                    g->set_line_width(2);
                } else {
                    g->set_color(30, 40, 51);  // Secondary roads
                    g->set_line_width(1);
                }
            }
        }
        // Draw road as a polyline
        LatLon start = getIntersectionPosition(segInfo.from);
        LatLon end = getIntersectionPosition(segInfo.to);
        ezgl::point2d startPt(x_from_lon(start.longitude()), y_from_lat(start.latitude()));
        ezgl::point2d endPt(x_from_lon(end.longitude()), y_from_lat(end.latitude()));
        for(int i = 0; i < segInfo.numCurvePoints; i++){
            LatLon curvePt = getStreetSegmentCurvePoint(segmentId, i);
            ezgl::point2d curvePoint(x_from_lon(curvePt.longitude()), y_from_lat(curvePt.latitude()));
            g->draw_line(startPt, curvePoint);
            startPt = curvePoint;
        }
        g->draw_line(startPt, endPt);
    }
}

//function for drawing the POIS
void drawPOIs(ezgl::renderer *g){
    if (!showPOI)
    {
        return;
    }
    
    //<a href="https://www.flaticon.com/free-icons/poi" title="poi icons">Poi icons created by Muhammad_Usman - Flaticon</a>
    poi_icon = g->load_png("libstreetmap/resources/point_of_interest.png");
    
    //<a href="https://www.flaticon.com/free-icons/restaurant" title="restaurant icons">Restaurant icons created by Freepik - Flaticon</a>
    rest_icon = g->load_png("libstreetmap/resources/cutlery.png");
    
    //<a href="https://www.flaticon.com/free-icons/hospital" title="hospital icons">Hospital icons created by Freepik - Flaticon</a>
    hosp_icon = g->load_png("libstreetmap/resources/hospital.png");
    
    //<a href="https://www.flaticon.com/free-icons/rental" title="rental icons">Rental icons created by surang - Flaticon</a>
    bike_icon = g->load_png("libstreetmap/resources/bicycle_rent.png");
    
    //<a href="https://www.flaticon.com/free-icons/atm-machine" title="atm machine icons">Atm machine icons created by Acellalelo - Flaticon</a>
    atm_icon = g->load_png("libstreetmap/resources/atm-machine.png");
    int scalingFac = 2000000/g->get_visible_world().area();
    int iconFac = std::min(scalingFac, 5);
    for (size_t i=0; i<POIs.size();i++){
        if (g->get_visible_world().area() < 2000000 && g->get_visible_world().contains(POIs[i])){
            if (getPOIType(i) == "restaurant" || getPOIType(i) == "fast_food"){
                g->draw_surface(rest_icon,POIs[i], 0.01*iconFac);
            }
            else if (getPOIType(i) == "clinic" || getPOIType(i) == "doctors" || getPOIType(i) == "hospital"){
                g->draw_surface(hosp_icon,POIs[i], 0.01*iconFac);
            }
            else if (getPOIType(i) == "bicycle_rental"){
                g->draw_surface(bike_icon,POIs[i], 0.03*iconFac);
            }
            else if (getPOIType(i) == "atm"){
                g->draw_surface(atm_icon,POIs[i], 0.05*iconFac);
            }
            else{
                g->draw_surface(poi_icon,POIs[i], 0.01*iconFac);
            }
            if (scalingFac>=50){
                g->set_color(0,0,0);
                g->set_font_size(10.0);
                ezgl::point2d textPos (POIs[i].x, POIs[i].y-3000/zoomLevel);
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
void drawRiverNames(ezgl::renderer *g) {
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
            if(nightmode == true){
                g->set_color(ezgl::WHITE);
            }
            else{
                g->set_color(ezgl::BLACK);
            }
            g->set_font_size(fontSize);
            g->set_text_rotation(angle);
            g->draw_text(mid, featureName);
        }
    }
}

//function for drawing the feature names
void drawFeatureNames(ezgl::renderer *g) {
    if (zoomLevel < 100) return;

    for (FeatureIdx i = 0; i < getNumFeatures(); i++) {
        FeatureType type = getFeatureType(i);

        //Conditions for deciding whether drawing the building
        if (type == ISLAND || type == STREAM) continue;
         if ( showstreetname && type==BUILDING) continue;
        if ( !showBuildingname && type==BUILDING) continue; 
        std::string featureName = getFeatureName(i);
        if (featureName.empty() || featureName == "<noname>") continue;

        double featureArea = findFeatureArea(i);
        if (featureArea < 100) continue;  

        ezgl::point2d center = findLargestInscribedRectangle(i);
        if(nightmode == true){
            g->set_color(ezgl::WHITE);
        }
        else{
            g->set_color(ezgl::BLACK);
        }
        g->set_font_size(fontSize);
        g->draw_text(center, featureName);
    }
}

//Function for highlighting the intersections
void drawIntersectionHighlight(ezgl::renderer *g){
    for(int i=0; i<intersections.size(); i++){
        if(intersections[i].highlight && zoomLevel > 5){
            g->set_color(255,0,0);
            g->fill_arc(ezgl::point2d(x_from_lon(intersections[i].pos.longitude()), y_from_lat(intersections[i].pos.latitude())), 500/zoomLevel, 0, 360);
        }
    }
}

// Finds the largest inscribed rectangle inside a closed feature
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

//function for drawing street names 
void drawStreetNames(ezgl::renderer *g) {
    if (!showstreetname || zoomLevel < 165) return;  // Skip rendering at low zoom levels

    g->set_font_size(fontSize);
    if(nightmode == true){
        g->set_color(ezgl::WHITE);
    }
    else{
        g->set_color(ezgl::BLACK);
    }

    // **Always draw highways if zoom level is at least 100**
    if (zoomLevel >= 165) {
        for (const RoadLabel &road : highways) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }

    // **Draw main roads if zoom level is at least 166**
    if (zoomLevel >= 165) {
        for (const RoadLabel &road : main_roads) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }

    // **Draw secondary roads if zoom level is at least 25000**
    if (zoomLevel >= 1500) {
        for (const RoadLabel &road : secondary) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }

    // **Draw minor roads if zoom level is at least 5000**
    if (zoomLevel >= 2000) {
        for (const RoadLabel &road : minor) {
            g->set_text_rotation(road.angle);
            g->draw_text(road.position, road.name);
        }
    }
}

//function for preprocessing the road data （helped by GPT）
void pre_load_road_data() {
    highways.clear();
    main_roads.clear();
    secondary.clear();
    minor.clear();
    buildIntersectionStreetSegmentMap();


    for (StreetIdx street_id = 0; street_id < getNumStreets(); ++street_id) {
        const auto& segment_ids = streetSegmentVector[street_id];
        if (segment_ids.empty()) continue;

        // **Get Street Name and Skip Empty or "<unknown>" Names**
        std::string street_name = getStreetName(street_id);
        if (street_name.empty() || street_name == "<unknown>") {
            continue;
        }

        // **Compute Total Street Length**
        double total_length = 0.0;
        for (StreetSegmentIdx seg_id : segment_ids) {
            total_length += findStreetSegmentLength(seg_id);
        }

        // **Ignore streets shorter than 200m**
        if (total_length < 200) continue;

        // **Classify Road by Length**
        int road_class = -1;
        if (total_length >= 1300) {
            road_class = 3;
        } else if (total_length >= 800) {
            road_class = 2;
        } else if (total_length >= 400) {
            road_class = 1;
        } else {
            road_class = 0;
        }

        // **Label Every 200m**
        double accumulated_distance = 0.0;
        for (StreetSegmentIdx seg_id : segment_ids) {
            StreetSegmentInfo seg_info = getStreetSegmentInfo(seg_id);
            double segment_length = findStreetSegmentLength(seg_id);
            
            // **Convert LatLon to pixel coordinates**
            ezgl::point2d start = {
                x_from_lon(getIntersectionPosition(seg_info.from).longitude()),
                y_from_lat(getIntersectionPosition(seg_info.from).latitude())
            };

            ezgl::point2d end = {
                x_from_lon(getIntersectionPosition(seg_info.to).longitude()),
                y_from_lat(getIntersectionPosition(seg_info.to).latitude())
            };

            // **Label every 200m**
            while (accumulated_distance + 100 <= segment_length) {
                double ratio = (accumulated_distance + 200) / segment_length;
                ezgl::point2d label_pos = {
                    start.x + ratio * (end.x - start.x),
                    start.y + ratio * (end.y - start.y)
                };

                // **Compute Angle**
                double dx = end.x - start.x;
                double dy = end.y - start.y;
                double angle = atan2(dy, dx) * 180.0 / M_PI;
                if (angle < 0) angle += 180;  // Keep text upright

                // **Store Label**
                RoadLabel label = {label_pos, angle, street_name};

                // **Assign to Correct Road Type**
                switch (road_class) {
                    case 3: highways.push_back(label); break;
                    case 2: main_roads.push_back(label); break;
                    case 1: secondary.push_back(label); break;
                    case 0: minor.push_back(label); break;
                }

                accumulated_distance += 300;
            }

            accumulated_distance -= segment_length;  // Adjust for next segment
            if (accumulated_distance < 0) accumulated_distance = 0;
        }
    }
}

void preloadFeatureDrawingOrder() {
    sortedFeatureIndices.clear();

    // Temporary vector to store (area, feature index)
    std::vector<std::pair<double, FeatureIdx>> area_feature_pairs;

    for (FeatureIdx i = 0; i < getNumFeatures(); ++i) {
        int numPoints = getNumFeaturePoints(i);
        if (numPoints < 3) continue; // Must be a polygon

        // Compute polygon area (Shoelace formula in screen space)
        double area = 0.0;
        area = findFeatureArea(i);

        area_feature_pairs.emplace_back(area, i);
    }

    // Sort descending by area （helped by chatgpt）
    std::sort(area_feature_pairs.begin(), area_feature_pairs.end(),
              [](const auto &a, const auto &b) {
                  return a.first > b.first;
              });

    // Extract ordered indices
    for (const auto &pair : area_feature_pairs) {
        sortedFeatureIndices.push_back(pair.second);
    }
}

// Draws the computed path on the map using colored segments based on speed limits.
// Also renders directional arrows along the path.
void drawPath(ezgl::renderer *g) {
    if (currentPath.empty()) return;

    g->set_line_width(3);

    for (size_t i = 0; i < currentPath.size(); ++i) {
        StreetSegmentIdx segmentId = currentPath[i];
        StreetSegmentInfo seg = getStreetSegmentInfo(segmentId);
        std::vector<ezgl::point2d> points;
        
        //set the vector for drawing
        LatLon fromLL = getIntersectionPosition(seg.from);
        points.push_back({x_from_lon(fromLL.longitude()), y_from_lat(fromLL.latitude())});
        
        //Go through all the curve points
        for (int j = 0; j < seg.numCurvePoints; ++j) {
            LatLon curve = getStreetSegmentCurvePoint(segmentId, j);
            points.push_back({x_from_lon(curve.longitude()), y_from_lat(curve.latitude())});
        }

        LatLon toLL = getIntersectionPosition(seg.to);
        points.push_back({x_from_lon(toLL.longitude()), y_from_lat(toLL.latitude())});

        // Speed coloring
        float speed = seg.speedLimit;
        if (speed < 15) g->set_color(30, 144, 255);
        else if (speed < 30) g->set_color(138, 43, 226);
        else g->set_color(50, 205, 50);

        for (size_t j = 0; j < points.size() - 1; ++j) {
            g->draw_line(points[j], points[j + 1]);
        }
    }
    drawPathArrows(g);
}

// Returns the polyline (set of 2D points) representing the street segment.
// The points are ordered in the direction of travel: forward or reverse.
// This function is helped by chatgpt
std::vector<ezgl::point2d> getSegmentPolyline(StreetSegmentIdx segmentId, bool forward) {
    StreetSegmentInfo seg = getStreetSegmentInfo(segmentId);
    std::vector<ezgl::point2d> points;

    LatLon from = getIntersectionPosition(seg.from);
    LatLon to = getIntersectionPosition(seg.to);

    if (forward) {
        // Start from the 'from' intersection
        points.push_back({x_from_lon(from.longitude()), y_from_lat(from.latitude())});
        for (int i = 0; i < seg.numCurvePoints; ++i) {
            LatLon curve = getStreetSegmentCurvePoint(segmentId, i);
            points.push_back({x_from_lon(curve.longitude()), y_from_lat(curve.latitude())});
        }
        points.push_back({x_from_lon(to.longitude()), y_from_lat(to.latitude())});
    } else {
        // Start from the 'to' intersection (reverse direction)
        points.push_back({x_from_lon(to.longitude()), y_from_lat(to.latitude())});
        for (int i = seg.numCurvePoints - 1; i >= 0; --i) {
            LatLon curve = getStreetSegmentCurvePoint(segmentId, i);
            points.push_back({x_from_lon(curve.longitude()), y_from_lat(curve.latitude())});
        }
        points.push_back({x_from_lon(from.longitude()), y_from_lat(from.latitude())});
    }

    return points;
}

// Draws a directional arrow from point p1 to point p2 using a filled triangle.
// The arrow indicates travel direction along the path segment.
void drawArrow(ezgl::renderer *g, const ezgl::point2d& p1, const ezgl::point2d& p2) {
    double dx = p2.x - p1.x, dy = p2.y - p1.y;
    double norm = std::sqrt(dx * dx + dy * dy);
    if (norm < 1e-5) return;

    double ux = dx / norm, uy = dy / norm;
    double arrowLen = 1200.0 / zoomLevel;
    double arrowWidth = 750.0 / zoomLevel;

    ezgl::point2d mid((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    ezgl::point2d tip(mid.x + ux * arrowLen, mid.y + uy * arrowLen);
    ezgl::point2d left(mid.x - uy * arrowWidth, mid.y + ux * arrowWidth);
    ezgl::point2d right(mid.x + uy * arrowWidth, mid.y - ux * arrowWidth);

    g->fill_poly({left, tip, right});
}

// Draws directional arrows along each street segment in the current path.
// Arrows indicate travel direction and are spaced based on distance.
void drawPathArrows(ezgl::renderer *g) {
    if (currentPath.empty()) return;

    g->set_color(0, 0, 0);  // arrows are black
    double arrowSpacing = 15000.0; // meters
    double accumulatedDist = 0.0;

    for (size_t idx = 0; idx < currentPath.size(); ++idx) {
        StreetSegmentIdx segmentId = currentPath[idx];
        bool forward = true;

        // Determine actual direction
        if (idx > 0) {
            StreetSegmentInfo prev = getStreetSegmentInfo(currentPath[idx - 1]);
            StreetSegmentInfo curr = getStreetSegmentInfo(segmentId);
            if (prev.to == curr.to || prev.from == curr.to)
                forward = false;
        }
        
        

        std::vector<ezgl::point2d> polyline = getSegmentPolyline(segmentId, forward);
        //calculate the accumulated distance
        for (size_t i = 0; i + 1 < polyline.size(); ++i) {
            ezgl::point2d p1 = polyline[i], p2 = polyline[i + 1];
            double len = std::hypot(p2.x - p1.x, p2.y - p1.y);
            accumulatedDist += len;

            if (accumulatedDist >= arrowSpacing / zoomLevel) {
                accumulatedDist = 0;
                drawArrow(g, p1, p2);
            }
        }
    }
}

// Prints turn-by-turn navigation messages for the current path.
// For each change in street name, it determines the direction of the turn
// (left, right, or forward) and displays the message in the console and GUI.
void printTurnMessageIfAny(ezgl::application* app) {
    if (currentPath.size() < 2) return;

    std::stringstream ss;

    for (size_t i = 1; i < currentPath.size(); ++i) {
        StreetSegmentInfo prevSeg = getStreetSegmentInfo(currentPath[i - 1]);
        StreetSegmentInfo currSeg = getStreetSegmentInfo(currentPath[i]);

        double angle = findStreetSegmentTurnAngle(i - 1, i);
        if (angle == NO_ANGLE) continue;

        double angle_deg = angle * 180.0 / M_PI;
        std::string turn;

        if (angle_deg > 30 && angle_deg < 150) {
            if (angle_deg < 90)
                turn = "⬅️  Left Turn";
            else
                turn = "➡️  Right Turn";
        } else {
            turn = "⬆️  Forward";
        }

        // Find the junction (common intersection)
        IntersectionIdx junction;
        if (prevSeg.to == currSeg.from || prevSeg.to == currSeg.to)
            junction = prevSeg.to;
        else
            junction = prevSeg.from;

        std::string interName = getIntersectionName(junction);
        ss << turn << " at " << interName << "\n";
    }

    // upload into GtkTextView
    GtkTextView* textview = GTK_TEXT_VIEW(app->get_object("turn_textview"));
    GtkTextBuffer* buffer = gtk_text_view_get_buffer(textview);
    gtk_text_buffer_set_text(buffer, ss.str().c_str(), -1);
}

// Checks if a valid path exists in currentPath.
// If not, displays a GTK dialog informing the user that no path was found.
bool checkPath(ezgl::application* app) {
    if (currentPath.empty()) {
        GtkWidget *dialog = gtk_message_dialog_new(
            GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(app->get_object("MainWindow")))),
            GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE,
            "No path found between the two intersections."
        );
        gtk_window_set_title(GTK_WINDOW(dialog), "No Path");
        gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_destroy(dialog);
        return false;
    } else {
        return true;
    }
}

