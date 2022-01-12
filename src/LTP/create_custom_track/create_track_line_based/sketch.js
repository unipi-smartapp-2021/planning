//=========================================================================================//
//parameters for the script:
const width = 3;          //track width
const cone_density = 4;     //number of cones per side in each part of the track (each line)

const translate_x = 0;      //translate on x axis
const translate_y = 0;      //translate on y axis
const scale = 0.4;          //scale the map. The smaller the scale, the smaller the size of the map will be
//=========================================================================================//

//points contains the points the user define at the beginning of the procedure
let points = [];

//these arrays contain the points of yellow and blue cones
let yellow_cones = [];
let blue_cones = [];

//if false, it does not autocomplete the track
let auto_complete = false

//show length
let show_length = false

//computed is used to say to the script that yellow and blue cones have already been computed 
computed = false

//used to mantain the scale indicated by the user
scaled_width = width * (1 / scale)

//used to know whether the mouse is moved or not
old_mouse_x = 0
old_mouse_y = 0
counter_same_mouse = 0

//checkboxes
let checkbox_autocomplete
let checkbox_length
let length_segments_input
let num_segments_input
let track_name_input

//constant for upper box
const height_upper_rect = 45
const pos_h_upper_rect = 18

const random_button_pos = 300
const random_area_length = 280

//constant for save area
const save_area_start = 580
const save_button_pos = 850

//constant for the canvas
const canvas_x = 1000
const canvas_y = 1000

//change coordinated of the point to be in the coordinate system defined by the user
function to_custom_coordinate_system(point) {
  return {
    x: (point.x + translate_x) * scale,
    y: (point.y + translate_y) * scale,
  }
}

function setup() {
  //button to save
  undo_button = createButton('undo');
  undo_button.position(10, pos_h_upper_rect);
  undo_button.mousePressed(undo)

  undo_button.style("border-radius", "4px")
  undo_button.style("padding", "4px 4px")

  checkbox_autocomplete = createCheckbox('auto-complete', false);
  checkbox_autocomplete.changed(CheckedEventAutoCompletion);
  checkbox_autocomplete.position(30 + undo_button.width, pos_h_upper_rect - 8);

  checkbox_length = createCheckbox('show lines', false);
  checkbox_length.changed(CheckedEventShowLength);
  checkbox_length.position(30 + undo_button.width, pos_h_upper_rect + 12);

  //random area
  random_button = createButton('generate');
  random_button.position(random_button_pos - 10, pos_h_upper_rect);
  random_button.mousePressed(random_track)

  random_button.style("border-radius", "4px")
  random_button.style("padding", "4px 4px")

  length_segments_input = createInput('150')
  length_segments_input.position(random_button_pos + random_button.width, pos_h_upper_rect + 5);

  length_segments_input.style("width", "5%")
  length_segments_input.style("padding", "3px 3px")
  length_segments_input.style("margin", "1px 0")

  num_segments_input = createInput('10')
  num_segments_input.position(random_button_pos + random_button.width + 100, pos_h_upper_rect + 5);

  num_segments_input.style("width", "5%")
  num_segments_input.style("padding", "3px 3px")
  num_segments_input.style("margin", "1px 0")

  //save area
  track_name_input = createInput(random(names))
  track_name_input.position(save_area_start, pos_h_upper_rect + 5);

  track_name_input.style("width", "8%")
  track_name_input.style("padding", "3px 3px")
  track_name_input.style("margin", "1px 0")


  save_json_button = createButton('save json');
  save_json_button.position(save_button_pos, pos_h_upper_rect);
  save_json_button.mousePressed(sv_json)

  save_json_button.style("border-radius", "4px")  
  save_json_button.style("padding", "4px 4px")

  save_yaml_button = createButton('save yaml');
  save_yaml_button.position(save_button_pos + save_json_button.width, pos_h_upper_rect);
  save_yaml_button.mousePressed(sv_yaml)

  save_yaml_button.style("border-radius", "4px")
  save_yaml_button.style("padding", "4px 4px")



  frameRate(10);

  createCanvas(canvas_x, canvas_y);
}

function draw() {
  background(110);

  //print top rectangle
  fill(color('#64CA5F'));
  stroke("#081b1f");
  rect(0, 0, 1000, height_upper_rect);

  //save area
  fill(color('#F13818'));
  rect(500, 0, 500, height_upper_rect);
  fill(0, 0, 0);
  textSize(12);
  text('track name', save_area_start - 5, 12);

  //random area
  fill(color('#B6AAE7'));
  rect(random_button_pos - 20, 0, random_area_length, height_upper_rect);
  fill(0, 0, 0);
  textSize(12);
  text('length segment', random_button_pos + 60, 12);
  text('num. segments', random_button_pos + 160, 12);

  //MOUSE//**//

  textSize(16);

  //print position mouse
  if (mouseY > height_upper_rect) {
    fill("#081b1f");

    mouse_position = to_custom_coordinate_system({ x: mouseX, y: mouseY })

    //print text position of the mouse w.r.t. translate and scale options
    noStroke()
    text("(" + mouse_position.x + ", " + mouse_position.y + ")", mouseX, mouseY);

    //draw additional lines
    if (show_length) {
      stroke("#c0c0c0");
      let x1 = points[points.length - 1].x;
      let y1 = points[points.length - 1].y;
      let x2 = mouseX;
      let y2 = mouseY;

      function write_along_line(x1, y1, x2, y2) {
        new_point_1 = to_custom_coordinate_system({ x: x1, y: y1 })
        new_point_2 = to_custom_coordinate_system({ x: x2, y: y2 })
        let d = dist(new_point_1.x, new_point_1.y, new_point_2.x, new_point_2.y);
        push();
        translate((x1 + x2) / 2, (y1 + y2) / 2);
        rotate(atan2(y2 - y1, x2 - x1));
        text(nfc(d, 1), 0, -5);
        pop();
      }

      if (show_length) {
        write_along_line(x1, y1, x2, y1);
        write_along_line(x2, y1, x2, y2);
      }

      line(x1, y1, x2, y1)
      line(x2, y1, x2, y2)
    }

    if ((points.length == 1 || (points.length > 1 && counter_same_mouse <= 2))) {
      stroke("#c0c0c0");
    } else {
      stroke("#4F4C4B");
    }

    //draw line between last point drawn and position of the mouse
    if (points.length >= 1) {
      line(points[points.length - 1].x, points[points.length - 1].y, mouseX, mouseY)
    }

    //draw circle in mouse position
    circle(mouseX, mouseY, 5)
  }


  if (counter_same_mouse <= 2) {
    stroke("#c0c0c0");
    fill("#c0c0c0");
  } else {
    stroke("#4F4C4B");
    fill("#4F4C4B");
  }

  //print the circles in the point clicked by the mouse 
  points.forEach((pt, i) => circle(pt.x, pt.y, 5));

  //if it has at least two points, the program build the lines between the points
  if (points.length > 1) {
    for (let i = 0; i < points.length - 1; i++) {
      line(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
    }
  }

  //to check wheter the mouse is moved or not
  if (old_mouse_x == mouseX && old_mouse_y == mouseY) {
    counter_same_mouse += 1
  } else {
    //we reset the counter that counts how many times the mouse wasnt moved
    counter_same_mouse = 0
  }

  //update mouse values
  old_mouse_x = mouseX
  old_mouse_y = mouseY

  //CONES//**//

  //we compute the cones once we have at least two cones. The computed flag is used to not compute the cones continuously
  if (points.length >= 2 && !computed) {
    yellow_cones = []
    blue_cones = []

    if (auto_complete == true) {
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
  if (mouseY > height_upper_rect)
    points.push({ x: mouseX, y: mouseY });
  else
    return true
  // prevent default
  computed = false
  return false;
}

function CheckedEventAutoCompletion() {
  if (checkbox_autocomplete.checked()) {
    auto_complete = true
  } else {
    auto_complete = false
  }
  computed = false
}

function CheckedEventShowLength() {
  if (checkbox_length.checked()) {
    show_length = true;
  } else {
    show_length = false;
  }
}

function undo() {
  points.pop();
  computed = false
}

function random_track() {
  numSeg = parseInt(num_segments_input.value());
  len = parseInt(length_segments_input.value());
  min_x = 25;
  max_x = canvas_x - 25;
  min_y = height_upper_rect + 25;
  max_y = canvas_y - 25;

  const MAX_FAILS = 250;
  let fail = false

  if (numSeg > 20)
    numSeg = 20;

  //compute first two points randomly
  points_random = [
    {
      x: random(min_x, max_x),
      y: random(min_y, max_y)
    }
  ];
  const last_point = points_random.at(-1)
  points_random.push({
    x: random(Math.max(last_point.x - len, min_x), Math.min(last_point.x + len, max_x)),
    y: random(Math.max(last_point.y - len, min_y), Math.min(last_point.y + len, max_y))
  });

  for (i = 1; i < numSeg; i++) {
    const last_point = points_random.at(-1)
    const penultimate_point = points_random.at(-2)
    let vector_last_point = createVector(
      last_point.x - penultimate_point.x,
      last_point.y - penultimate_point.y
    );

    new_point_found = false

    let new_point
    let counter_fail = 0
    while (!new_point_found && counter_fail < MAX_FAILS) {
      new_point = {}
      new_point.x = random(Math.max(last_point.x - len, min_x), Math.min(last_point.x + len, max_x))
      new_point.y = random(Math.max(last_point.y - len, min_y), Math.min(last_point.y + len, max_y))

      if (dist(new_point.x, new_point.y, last_point.x, last_point.y) > len / 2) {

        let new_point_vector = createVector(
          new_point.x - last_point.x,
          new_point.y - last_point.y
        );

        let angleBetween = vector_last_point.angleBetween(new_point_vector);
        if (angleBetween >= -1.3 && angleBetween <= 1.3) {
          new_point_found = true;
          for (j = 0; j < points_random.length - 2; j++) {
            point1 = points_random[j];
            point2 = points_random[j + 1];
            if (intersectionPt(point1.x, point2.x, last_point.x, new_point.x, point1.y, point2.y, last_point.y, new_point.y)) {
              new_point_found = false;
              console.log("found intersection")
              break;
            }
          }
        }
      }

      counter_fail += 1
    }

    if (counter_fail == MAX_FAILS){
      console.log("Failed generating track. trying again");
      fail = true;;
      break;
    }

    points_random.push(new_point);
  }

  if (fail){
    random_track()
  } else 
    points = points_random;
    track_name_input.value(random(names))

  computed = false
}