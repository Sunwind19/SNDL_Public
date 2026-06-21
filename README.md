# SNDL
This is repository for SNDL, a biomimetic robot that is your unique companion for your climate action. When you pick up, SNDL carries! 

## Project Introduction
<img width="648" height="510" alt="image" src="https://github.com/user-attachments/assets/42f25f23-4353-422d-8615-7f70c1faa318" />
<img width="898" height="796" alt="image" src="https://github.com/user-attachments/assets/c017f482-18e5-4c8b-baf2-b75505830295" />
<img width="1458" height="1444" alt="image" src="https://github.com/user-attachments/assets/62ab1243-b989-4798-a1d1-5065458d1f3b" />


This is SNDL, a mountain waste collecting beetle-like 6-legged biomimetic robot companion, building a sustainable future through protecting nature closest to our daily life. 

Can you believe everyday trash on mountains causes biodiversity loss, water contamination, and economic loss of $16M per 100t? Trash is piling up every second – 2,200tons in Yosemite – as rugged terrain limits humans to collect 4% of total trash. 

SNDL revolutionize this reality through first-ever innovation inspired by insects’ skeletal structures evolved to survive in mountains. Built over believing “nature already holds the answer to our problem”, SNDL targets to remove all the accumulated trash through collaboration with human. No need to carry heavy wastes and get exhausted over exhausted: you can just hand them in to SNDL and it’ll carry it for you!

By mimicking the Diabolical Ironclad Beetle and the Metallifer Stag Beetle, SNDL plan to restore one mountain ecosystem — one BIOME — proving we aren’t late to save this planet.

## Zine page
<img width="652" height="936" alt="image" src="https://github.com/user-attachments/assets/cb69e9fc-8993-4f98-8f22-e5262ebff3be" />


## BOM
The key parts here were
-2 PCA boards
-1 ESP Devkit V4
-18 MG90S motors
-20A DC-DC & 2A Dual Buck converter system
-15A & 3A Dual
You can check out the details in BOM (Bill of Materials for SNDL).csv file and get the materials, and follow the schematics.pdf file! Soldering is fine, but I recommend using wago connectors at the beginning since theres high chance motor might burn for beginners. 

## Key CAD file
The most important design choice I made was to mimic Diabolical Ironclad beetle's exoskeleton into the robot's main frame, so that it can carry a load of waste at once and be perfect companion for your waste-collecting journey. Specifically, diabolical ironclad beetle can endure 4900x of its body weight because of its suture! As shown in assembly body.f3z / .step file, this exoskeleton is covered by shell, where body-coxa bracket, bearing bracket, femur and tibia are attached. 

## Firmware
The firmware has inverse kinematics & tripod gait feature.
You can connect your nintendo joycon or any other controller bluepad32 supports!

## Why?
The precious mountains where I once shared precious memories are now covered with waste. Just like scenary in WALL-E. Why? Because we couldn't collect them again unlike when we spreaded them out. Here, I wanted to bring robotic system to foster climate action and prove its not too late. 
