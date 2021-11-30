# Tool for track generation


We developed two tools to generate a 2d track represented by an ordered sequence of yellow and blue cones: 
- line_based: you draw a set of lines, the tool generates the track along that line.
- point-based: you draw the points that represent either blue or yellow cones.

## Line Based
To initialize the tool, you need to click on the index file in the create_track_line_based directory. 
Then, clicking with the mouse on the grey canvas, the tool will start drawing points and lines between points. Once you are satisfied, you have to click ```create cones``` on the upper left of the canvas. The tool will generate the yellow and green cones smoothing the corners. Finally, you can save the track as a JSON file by clicking ```save```.

## Point Based
Again, click on the index file in the create_track_point_based directory. Then, click on the grey canvas to generate yellow cones. Drawn the yellow cones, it is time for the blue ones by clicking on ```change_color_cones``` button. Finally, you can save the result as a JSON file by clicking ```save```.