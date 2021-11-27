function setup() {
  createCanvas(400, 400);
}

data = '{"left_cones": {"x": [115.0, 111.73252989035961, 102.12423521448197, 86.74593080251337, 66.51121938178215, 42.62221576582357, 16.49813095944188, -10.30904110560719, -36.20672527575033, -59.656377752172034, -79.26488874308757, -93.86734509290838, -102.59623608071863, -104.93299097210054, -100.73878655321508, -90.26279441628824, -74.12737803725162, -53.29111906565255, -28.99186938124423, -2.6732121118538137, 24.1012995433623, 49.741030738337976, 72.72276228582244, 91.68118289673941, 105.49000034068611, 113.32885283134289], "y": [10.0, 36.61140851596345, 61.64187190644799, 83.60436669947441, 101.19413298105458, 113.36618828644991, 119.39740849051006, 118.92948756157274, 111.99022400234662, 98.99186938124421, 80.70663706551935, 58.2208261467985, 32.87028598995352, 6.161055362724902, -20.32010913986989, -45.000000000000014, -66.41242075048972, -83.28529057720684, -94.61621679246689, -99.73204552858066, -98.3288528313429, -90.49000034068612, -76.6811828967394, -57.7227622858224, -34.74103073833802, -9.101299543362341]}, "right_cones": {"x": [105.0, 102.02957262759965, 93.2947592858927, 79.31448254773943, 60.91929034707469, 39.20201433256688, 15.452846326765345, -8.917310096006535, -32.46065934159121, -53.7785252292473, -71.6044443118978, -84.8794046299167, -92.81476007338057, -94.93908270190957, -91.12616959383189, -81.60254037844386, -66.93398003386511, -47.9919264233205, -25.901699437494756, -1.9756473744125582, 22.364817766692997, 45.673664307579976, 66.56614753256585, 83.8010753606722, 96.3545457642601, 103.4807753012208], "y": [10.0, 34.19218955996678, 56.947156278589084, 76.91306063588583, 92.90375725550416, 103.96926207859083, 109.45218953682733, 109.02680687415703, 102.71838545667875, 90.90169943749474, 74.27876096865394, 53.83711467890773, 30.79116908177593, 6.510050329749911, -17.5637355816999, -40.000000000000014, -59.46583704589973, -74.80480961564258, -85.10565162951535, -89.75640502598242, -88.48077530122082, -81.35454576426011, -68.80107536067217, -51.56614753256582, -30.67366430758001, -7.36481776669304]}, "trajectory": {"positions": {"x": [110.0, 106.88105125897962, 97.70949725018733, 83.03020667512641, 63.715254864428424, 40.91211504919522, 15.975488643103613, -9.613175600806862, -34.33369230867077, -56.717451490709664, -75.43466652749268, -89.37337486141254, -97.7054980770496, -99.93603683700505, -95.93247807352348, -85.93266739736606, -70.53067903555836, -50.64152274448652, -27.446784409369492, -2.324429743133186, 23.233058655027648, 47.707347522958976, 69.64445490919414, 87.74112912870581, 100.9222730524731, 108.40481406628184], "y": [10.0, 35.40179903796511, 59.29451409251854, 80.25871366768013, 97.04894511827936, 108.66772518252037, 114.42479901366869, 113.97814721786489, 107.35430472951268, 94.94678440936949, 77.49269901708664, 56.02897041285311, 31.830727535864725, 6.335552846237406, -18.941922360784893, -42.500000000000014, -62.939128898194724, -79.04505009642472, -89.86093421099112, -94.74422527728154, -93.40481406628186, -85.92227305247312, -72.74112912870578, -54.64445490919411, -32.70734752295901, -8.23305865502769]}, "velocities": [14.415195273897272, 14.734959604098337, 14.642322030458732, 14.666774698437614, 14.660654067912985, 14.662077383539996, 14.661776520418492, 14.661830763669734, 14.661824158843908, 14.661823698916706, 14.661824411492947, 14.66182407692964, 14.661824189326202, 14.661824189326163, 14.661824076929719, 14.66182441149278, 14.661823698916807, 14.661824158843947, 14.661830763669608, 14.661776520418774, 14.66207738353964, 14.660654067913226, 14.66677469843746, 14.642322030458823, 14.734959604098316, 14.415195273897355]}}'

const data_json = JSON.parse(data)

const left_cones = data_json.left_cones
const right_cones = data_json.right_cones
const trajectory = data_json.trajectory

let step = -1
let alpha_step = 1

let tick = 0
let cycle = 0

let current_pos_x = null
let current_pos_y = null
let next_pos_x = null
let next_pos_y = null

function linearEase(alpha, start, end) {
  return alpha * (end - start) + start;
}

function draw() {
  translate(200, 200);  //moves the origin to bottom left
  scale(1, -1);  //flips the y values so y increases "up"
  
  background(220);
  
  fill(color('#fcc800'));
  noStroke();
  for (let i = 0; i < left_cones.x.length; i++){
    circle(left_cones.x[i], left_cones.y[i], 5)
  }
  
  fill(color('#0102fd'));
  noStroke();
  for (let i = 0; i < right_cones.x.length; i++){
    circle(right_cones.x[i], right_cones.y[i], 5)
  }
  
  fill(color('#e60201'));
  noStroke();
  
  if (alpha_step >= 1){
    //compute new step
    step = (step + 1)%trajectory.positions.x.length;
    
    //reset alpha
    alpha_step = 0;
    tick = 0;
    
    //compute current and next positions
    current_pos_x = trajectory.positions.x[step]
    current_pos_y = trajectory.positions.y[step]
    next_pos_x = trajectory.positions.x[(step + 1)%trajectory.positions.x.length]
    next_pos_y = trajectory.positions.y[(step + 1)%trajectory.positions.x.length]
    
    //compute step for the current passage
    distance = Math.sqrt(Math.pow(current_pos_x - next_pos_x,2) + Math.pow(current_pos_y - next_pos_y,2))
    
    //number of steps
    time_to_complete_trait = distance/(trajectory.velocities[step]/1.8)
    
    cycle = time_to_complete_trait*10
  }
  
  alpha_step = tick/cycle
  
  circle(linearEase(alpha_step, current_pos_x, next_pos_x), linearEase(alpha_step, current_pos_y, next_pos_y), 3)
  
  tick++

}