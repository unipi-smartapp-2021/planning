//=========================================================================================//
//parameters for the script:
const width = 1.5;          //track width
const cone_density = 2;     //number of cones per side in each part of the track (each line)

const translate_x = -500;   //translate on x axis
const translate_y = -500;   //translate on y axis
const scale = 0.2;          //scale the map. The smaller the scale, the smaller the size of the map will be
//=========================================================================================//

//points contains the points the user define at the beginning of the procedure
let points = [];

//these arrays contain the points of yellow and blue cones
let yellow_cones = [];
let blue_cones = [];

//if false, it does not autocomplete the track
let auto_complete = false

//it becomes true when the user presses the create_cones_button. 
//If true, the user can not add other points and the script builds the yellow and blue cones array
create_cones_activated = false
//computed is used to say to the script that yellow and blue cones have already been computed 
computed = false

scaled_width = width * (1 / scale)

old_mouse_x = 0
old_mouse_y = 0
counter_same_mouse = 0

let checkbox

function setup() {
  //button to save
  undo_button = createButton('undo');
  undo_button.position(10, 10);
  undo_button.mousePressed(undo)

  undo_button.style("border-radius", "4px")
  undo_button.style("padding", "4px 4px")

  //button to create cones
  //create_cones_button = createButton('create cones');
  //create_cones_button.position(20 + undo_button.width, 10);
  //create_cones_button.mousePressed(createCones);

  //create_cones_button.style("border-radius", "4px")
  //create_cones_button.style("padding", "4px 4px")

  checkbox = createCheckbox('auto-complete', false);
  checkbox.changed(CheckedEventAutoCompletion);
  checkbox.position(30 + undo_button.width, 10);

  save_button = createButton('save');
  save_button.position(300, 10);
  save_button.mousePressed(sv)

  save_button.style("border-radius", "4px")
  save_button.style("padding", "4px 4px")



  frameRate(10);

  createCanvas(1000, 1000);
}

function draw() {
  background(220);

  //print top rectangle
  fill(color('#a9a9a9'));
  rect(0, 0, 1000, 35);

  //print position mouse
  if (mouseY > 30) {
    fill("#081b1f");
    text("(" + ((mouseX + translate_x) * scale) + ", " + ((mouseY + translate_y) * scale) + ")", mouseX, mouseY);
    if (points.length == 1 || (points.length > 1 && counter_same_mouse <= 2))
      stroke("#081b1f");
    else 
      stroke("#c0c0c0");

    fill("#081b1f");
    if (points.length >= 1) {
      line(points[points.length - 1].x, points[points.length - 1].y, mouseX, mouseY)
    }
    circle(mouseX, mouseY, 5)
  }

  //print the circles in the point clicked by the mouse
  if (counter_same_mouse <= 2){
    stroke("#081b1f");
    fill("#081b1f");
  } else {
    stroke("#c0c0c0");
    fill("#c0c0c0");
  }

  points.forEach((pt, i) => circle(pt.x, pt.y, 5));

  //if it has at least two points, the program build the lines between the points
  if (points.length > 1 && !create_cones_activated) {
    for (let i = 0; i < points.length - 1; i++) {
      line(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
    }
  }
  
  if (old_mouse_x == mouseX && old_mouse_y == mouseY){
    counter_same_mouse += 1
    console.log(counter_same_mouse)
  } else {
    //we reset the counter that counts how many times the mouse wasnt moved
    counter_same_mouse = 0
  }

  old_mouse_x = mouseX
  old_mouse_y = mouseY

  //create_cones_activated becomes true only when the user presses the create_cones_button. 
  if (points.length >= 2 && !computed) {
    yellow_cones = []
    blue_cones = []

    if (auto_complete == true) {
      console.log(auto_complete)
      for (let i = 0; i < points.length; i++) {
        //select the points to build the curve
        let anchor_point_1 = points[(i - 1) % points.length];

        //if i-1 is negative, we are at the first element. Therefore, its anchor point is the last element in points array.
        if (i - 1 < 0) {
          anchor_point_1 = points[points.length - 1];
        }

        const point_1 = points[i];
        const point_2 = points[(i + 1) % points.length];
        const anchor_point_2 = points[(i + 2) % points.length];

        //print the curve
        //compute the yellow and blue cones
        get_cone_points(anchor_point_1, point_1, point_2, anchor_point_2, cone_density);
      }

      computed = true

    } else {
      //handle first point
      const anchor_point_1 = points[0]
      const point_1 = points[0];
      const point_2 = points[1];
      let anchor_point_2 = points[1];
      if (points.length > 2) {
        anchor_point_2 = points[2]
      }
      get_cone_points(anchor_point_1, point_1, point_2, anchor_point_2, cone_density);

      for (let i = 1; i < points.length - 2; i++) {
        //select the points to build the curve
        let anchor_point_1 = points[i - 1];
        const point_1 = points[i];
        const point_2 = points[i + 1];
        const anchor_point_2 = points[i + 2];
        get_cone_points(anchor_point_1, point_1, point_2, anchor_point_2, cone_density);
      }

      if (points.length > 2) {
        //handle last point
        const anchor_point_1 = points[points.length - 3]
        const point_1 = points[points.length - 2];
        const point_2 = points[points.length - 1];
        let anchor_point_2 = points[points.length - 1];
        get_cone_points(anchor_point_1, point_1, point_2, anchor_point_2, cone_density);
      }

      computed = true

    }

  }

  //print the cones
  fill(color('#EAED11'));
  noStroke();
  yellow_cones.forEach((pt, i) => circle(pt.x, pt.y, 1.2 * (1 / scale)));
  fill(color('#1E90FF'));
  noStroke();
  blue_cones.forEach((pt, i) => circle(pt.x, pt.y, 1.2 * (1 / scale)));
}

//compute the points for the yellow and the blue cones in the curve between p2 and p3 (p1 and p4 are used as anchor points)
function get_cone_points(p1, p2, p3, p4, steps) {
  for (let i = 0; i <= steps; i++) {
    let t = i / steps;
    let x = curvePoint(p1.x, p2.x, p3.x, p4.x, t);
    let y = curvePoint(p1.y, p2.y, p3.y, p4.y, t);
    let tx = curveTangent(p1.x, p2.x, p3.x, p4.x, t);
    let ty = curveTangent(p1.y, p2.y, p3.y, p4.y, t);
    let a = atan2(ty, tx);
    a -= PI / 2.0;
    yellow_cones.push({ x: - cos(a) * scaled_width + x, y: - sin(a) * scaled_width + y });
    blue_cones.push({ x: cos(a) * scaled_width + x, y: sin(a) * scaled_width + y });
  }
}

//called when a button of the mouse is pressed
function mousePressed() {
  if (!create_cones_activated)
    if (mouseY > 22)
      points.push({ x: mouseX, y: mouseY });
  // prevent default
  computed = false
  return false;
}

//triggered when the user clicked the create_cones button
function createCones() {
  //delete last point added
  create_cones_activated = true;
  return false;
}

function CheckedEventAutoCompletion() {
  console.log(checkbox.checked())
  if (checkbox.checked()) {
    auto_complete = true
  } else {
    auto_complete = false
  }
  computed = false
}

function undo() {
  points.pop()
}

//save yellow and blue cones as a JSON file
function sv() {
  //creating final result
  const res = { yellow_cones: null, blue_cones: null };
  res.yellow_cones = JSON.parse(JSON.stringify(yellow_cones))
  res.yellow_cones.map(pt => { pt.x = (pt.x + translate_x) * scale; pt.y = (pt.y + translate_y) * scale; return pt });
  res.blue_cones = JSON.parse(JSON.stringify(blue_cones))
  res.blue_cones.map(pt => { pt.x = (pt.x + translate_x) * scale; pt.y = (pt.y + translate_y) * scale; return pt });

  const doc = document.createElement("a");
  const file = new Blob([JSON.stringify(res, null, '\t')], { type: 'text/plain' });
  doc.href = URL.createObjectURL(file);
  doc.download = 'track.json';
  doc.click();
}