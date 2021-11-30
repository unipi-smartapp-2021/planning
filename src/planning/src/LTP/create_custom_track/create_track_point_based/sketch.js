function setup() {
  //button to change the color
  change_color_button = createButton('change_cone_color');
  change_color_button.position(0, 0);
  change_color_button.mousePressed(changeColor);
  //button to save
  save_button = createButton('save');
  save_button.position(0 + change_color_button.width, 0);
  save_button.mousePressed(sv)

  createCanvas(1000, 1000);
}

//data structure containing the result
let points_yellow = []
let points_blue = []
let cur_color = 0 //0 = yellow, 1=green

function draw() {
  background(100);

  //color yellow
  fill(color('#EAED11'));
  noStroke();
  //print yellow cones
  points_yellow.forEach((pt, i) => ellipse(pt.x, pt.y, 10, 10));

  //color blue
  fill(color('#1E90FF'));
  noStroke();
  //print blue cones
  points_blue.forEach((pt, i) => ellipse(pt.x, pt.y, 10, 10));
}

//called when mouse is pressed
function mousePressed() {
  //add point only if the mouse is under the button area (to avoid to create a point when user click the button)
  if (mouseY > 20) {
    if (cur_color == 0)
      points_yellow.push({ x: mouseX, y: mouseY })
    else
      points_blue.push({ x: mouseX, y: mouseY })
  }
  // prevent default
  return false;
}

//change the color drawn
function changeColor() {
  cur_color = (cur_color + 1) % 2
  // prevent default
  return false;
}

//save the result in a JSON
function sv() {
  //creating final result
  const res = { yellow_cones: null, green_cones: null };
  res.yellow_cones = points_yellow;
  res.green_cones = points_blue;

  const doc = document.createElement("a");
  const file = new Blob([JSON.stringify(res)], { type: 'text/plain' });
  doc.href = URL.createObjectURL(file);
  doc.download = 'track.json';
  doc.click();
}
