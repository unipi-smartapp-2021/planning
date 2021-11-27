//=========================================================================================//
//parameters for the script:
const width =10;          //track width
const cone_density = 5;   //number of cones per side in each part of the track (each line)
//=========================================================================================//

function setup() {
  //button to create cones
  create_cones_button = createButton('create cones');
  create_cones_button.position(0, 0);
  create_cones_button.mousePressed(createCones);

  //button to save
  save_button = createButton('save');
  save_button.position(0 + create_cones_button.width, 0);
  save_button.mousePressed(sv)

  createCanvas(1000, 1000);
}

//points contains the points the user define at the beginning of the procedure
let points = [];

//these arrays contain the points of yellow and blue cones
let yellow_cones = [];
let blue_cones = [];

//it becomes true when the user presses the create_cones_button. 
//If true, the user can not add other points and the script builds the yellow and blue cones array
create_cones_activated = false
//computed is used to say to the script that yellow and blue cones have already been computed 
computed = false

function draw() {
  background(220); 
  noFill();

  //print the circles in the point clicked by the mouse
  points.forEach((pt, i) => circle(pt.x, pt.y, 5));
  
  //if it has at least two points, the program build the lines between the points
  if (points.length > 1 && !create_cones_activated){
    for (let i = 0; i < points.length-1; i++){
      line(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
    }
  }
  
  //create_cones_activated becomes true only when the user presses the create_cones_button. 
  if (create_cones_activated){

    for (let i = 0; i < points.length; i++){
      //select the points to build the curve
      let anchor_point_1 = points[(i-1)%points.length];

      //if i-1 is negative, we are at the first element. Therefore, its anchor point is the last element in points array.
      if (i-1 < 0){
        anchor_point_1 = points[points.length - 1];
      }

      const point_1 = points[i];
      const point_2 = points[(i+1)%points.length];
      const anchor_point_2 = points[(i+2)%points.length];

      //print the curve
      curve(anchor_point_1.x, anchor_point_1.y, point_1.x, point_1.y, point_2.x, point_2.y, anchor_point_2.x, anchor_point_2.y);
      if (!computed){
        //compute only once the yellow and blue cones
        get_cone_points(anchor_point_1, point_1, point_2, anchor_point_2, cone_density);
      }
    }
    computed = true
    console.log(yellow_cones.length, blue_cones.length)
  }
  
  //print the cones
  if (create_cones_activated){
    fill(color('#EAED11'));
    noStroke();
    yellow_cones.forEach((pt, i) => circle(pt.x, pt.y, 5));
    fill(color('#1E90FF'));
    noStroke();
    blue_cones.forEach((pt, i) => circle(pt.x, pt.y, 5));
  }
}

//compute the points for the yellow and the blue cones in the curve between p2 and p3 (p1 and p4 are used as anchor points)
function get_cone_points(p1, p2, p3, p4, steps){  
  for (let i = 0; i <= steps; i++) {
    let t = i / steps;
    let x = curvePoint(p1.x, p2.x, p3.x, p4.x, t);
    let y = curvePoint(p1.y, p2.y, p3.y, p4.y, t);
    let tx = curveTangent(p1.x, p2.x, p3.x, p4.x, t);
    let ty = curveTangent(p1.y, p2.y, p3.y, p4.y, t);
    let a = atan2(ty, tx);
    a -= PI / 2.0;
    yellow_cones.push({x:- cos(a) * width + x,y: - sin(a) * width + y});
    blue_cones.push({x: cos(a) * width + x,y: sin(a) * width + y});
  }
}

//called when a button of the mouse is pressed
function mousePressed(){
  if (!create_cones_activated)
    points.push({x: mouseX, y: mouseY});
  // prevent default
  return false;
}

//triggered when the user clicked the create_cones button
function createCones(){
  //delete last point added
  create_cones_activated = true;
  return false;
}

//save yellow and blue cones as a JSON file
function sv(){
//creating final result
const res = {yellow_cones: null, blue_cones:null};
res.yellow_cones = yellow_cones;
res.blue_cones = blue_cones;

const doc = document.createElement("a");
const file = new Blob([JSON.stringify(res, null, '\t')], {type: 'text/plain'});
doc.href = URL.createObjectURL(file);
doc.download = 'track.json';
doc.click();
}