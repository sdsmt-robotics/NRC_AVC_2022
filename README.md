![alt text](https://github.com/sdsmt-robotics/NRC_AVC_2022/blob/main/header.png "Header")
# Goal of this Repo
The goal of this repo is to store the programs needed to replicate the results from the 2022 AVC car. The files in this repository will either be under the MIT or GPL v3 Licenses. However, we ask that these files in their entirity (individual files are ok) do not be shared since they make up the bulk of the work when it comes to the AVC. Thank you for your understanding.

# The Challenge
The following is taken from the Contest Manual:

>Contest Description
>For the Autonomous Vehicle Challenge each team will design and build a vehicle to navigate an obstaclecourse. A successful run is one where the vehicle navigates around the 4 waypoints (blue stanchions) andcrosses the finish line in under 5 minutes. Additionally, bonus points are given for completing special tasksduring a run. Once earned, bonus points cannot be taken away. Teams can score bonus points even if theydo not complete a successful run.
>Rules and Course Layout
>1. The course will be located on the parking lot of the Marion Veterans Memorial Coliseum, Marion, Ohio
>2. Each vehicle must pass on the outside of stanchions in the course shown in Figure 1: AVC CourseLayout. Blue stanchions will be placed on top of the blue corner dots shown and yellow stanchionson the yellow dot. Final placements of stanchions and obstacles will be at the discretion of thejudges.
>3. The autonomous vehicle must fit inside a 24” x 24” x 24” space. Any robot entered that does notmeet the size requirement by the end of the device evaluation or expands beyond that size duringcompetition will be disqualified.
>4. The vehicle must be fully autonomous and self-contained. No transmitters or communicationbeacons (other than GPS) of any kind are allowed. You may NOT tether to a laptop or other device. Everything necessary for the vehicle’s navigation/processing/sensing must be attached and part ofthe vehicle itself.
>5. The AVC event will take place outside in the parking lot west of Veterans Memorial Coliseum. The event will run regardless of weather conditions. (with the exception of lightning)
>6. Allowances for unforeseen delays will be taken into account, but will be up to the contest judges. Please notify the contest judges immediately if you have an issue that prevents you from competingon schedule. (Repairs, bad code, gremlins, or dead batteries do not count. Be prepared for anything and everything.)
>7. Each team is given 3 attempts throughout the day to earn points.
>8. Teams will be called when it is time to bring your vehicle to the starting line and if you are not ready, you will forfeit that run and receive no points.
>9. During the competition the course will be closed to both spectators and participants. Participants with vehicles currently running may follow within the inner perimeter of the course (Yellow lines in figure 1), but may not be on the course with the vehicles when they are running.
>10. Teams are expected to make all necessary measurements, adjustments, and sensor readings before the event starts. Once the first heat is started, only competing vehicles and their team may be in the course, and to start their vehicle for the run.
>11. We will be running 1 vehicle at a time. You will place your robot anywhere behind the start/finish line, wait for a signal, and press a button to start your robot. Time will begin when the robot first crosses the Start line. It must be in a ready position, and be started with a physical input (button, switch, etc). It may not be started wirelessly.
>12. If you need to wait for GPS lock or a setup routine, you will need to do this before your run starts.
>13. Time-based points start at 300 and are deducted (1 per second) until the run is completed. You cannot get negative points for time. Time points only count if your run is successful. Each successful run will be given points based on the time it took them to finish the course and whatever bonuses were achieved.
>14. All teams are scored and ranked by the total number of points accrued in all three runs. The team with the highest points wins. Each run will add more points to the team’s overall score.
>Bonus Points
>* 50 - passing through the arch
>* 50 - clearing the ramp
>* 25 - each successful corner cleared (vehicle must completely clear corner, not just reach it)
>Scoring Examples:
>* Teams complete a run in 2.5 minutes and passes under hoop. this team would score 150 points for their time, 20 points for each completed corner (100 points), and 50 points for the hoop bonus. Their total score would be 150+100+50 = 300
>* Team does not complete course, but goes around 3 corners and clears the hoop and ramp. they would score no points for time (did not complete course), but would receive 60 points for 3 successful corners, 50 points for the hoop, and 50 points for the ramp. they would end up with a total of 160 points for the run.
>* Team completes course in 1 minute, but does not clear the ramp or the hoop. They would get 240 points for time and 100 points for successfully clearing all 5 corners. Their total score 

![alt text](https://github.com/sdsmt-robotics/NRC_AVC_2022/blob/main/AVC_Course.png "AVC Course")
                                                                           
Reference: https://www.thenrc.org/uploads/7/1/5/1/71512601/nrccontestrules2022.pdf

# Documentation (WIP)
## Blob Detection
The blob detection algorithm does the following:

Input: USB Camera Video, Scaling Factor

Return: Blob Size, Turning Angle to Blob, Bucket Color {'Blue': 0, 'Yellow': 1, 'Red': 2}

Basic Process:
* Scale down the Video
* Create masks of given color ranges (Blue, Yellow, and Red in this case)
* Load SimpleBlobDetector and filter by area, color, and circularity
* Calculate blob size and approximate turning angle to blob

## Unity Simulation
Launch files in Unity 2020.3.27f1 or newer using Ubuntu 20.04 or Windows 10 (why would you do that to yourself!)
