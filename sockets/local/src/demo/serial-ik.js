// Adapted from:
// https://slides.com/hugohadfield/game2020 page 24.

import Algebra from "ganja.js";
import { replaceCanvas } from "../util";

export function serial_ik() {
  // Create a Clifford Algebra with 4,1 metric for 3D CGA. 
  Algebra(4,1,() => { 

    // We start by defining a null basis, and upcasting for points
    var ni = 1e4 + 1e5, no = 0.5e5 - 0.5e4;
    var up = (x) => no + x + 0.5 * x * x *ni;
    var down = (x) => (x ^ (no^ni)) * (no^ni) / (-x|ni);
    
    // Set the lengths of the robot links
    var rho = 0.8;
    var l = 0.6;
    
    // Choose a beginning starting target point
    var target = up(0.6 * 1e1 + 0.8 * 1e2 + 0.0 * 1e3); // y (1e2) toward up
    
    // Draw a robot base as a grade-3 circle
    var Cbase = (up(0.2 * 1e1) ^ up(0.2 * 1e3) ^ up(-0.2 * 1e1)).Normalized;
    
    // Graph the items.  
    // document.body.appendChild(this.graph(()=> {
    replaceCanvas(this.graph(()=>{
    // let myDiv = document.getElementById("my-div");
    // myDiv.setAttribute('height', '100');
    // myDiv.appendChild(this.graph(()=> {
    
    // The actual inverse kinematics of the robot
    var S0 = !(up(0) - 0.5 * (rho ** 2) * ni); // grade-4 sphere
    var S1 = !(target - 0.5 * (l ** 2) * ni); // grade-4 sphere
    var C = S0 & S1; // grade-3 circle
    var P = target ^ up(0) ^ up(1e2) ^ ni; // grade-4 plane
    var PP = -(P & C).Grade(2).Normalized; // grade-2 point pair
    
    // The square of the point pair describes if the spheres intersect
    if ((PP * PP)[0] > 0){ 
    // If the spheres intersect then we can just choose one solution
    var elb = (1 + PP * (1/Math.sqrt((PP * PP)[0]))) * (PP | ni); // by method of projectors
    // var elb = (1 - PP * (1/Math.sqrt((PP * PP)[0]))) * (PP | ni); // by method of projectors
    
    var endpoint = target;
    }
    else{
    // If the spheres do not intersect then we will just reach for the object.
    var endpoint = up((rho + l) * down(target).Normalized);
    var elb = up(rho * down(target).Normalized);
    }
    
    // Solve for IK position problem
    // Form lines
    var L1 = (elb ^ target ^ ni).Normalized; // grade-3 line
    var L2 = (elb ^ no ^ ni).Normalized; // grade-3 line
    var L1proj = ((L1 | (Cbase ^ ni)) | (Cbase ^ ni)).Normalized; // grade-3 line (project L1 onto base plane)
    var downt = down(target).Normalized;
    
    // Solve for joint variables
    var theta0 = Math.atan2((downt | 1e3)[0], (downt | 1e1)[0]);
    var theta1 = Math.acos((L2 | L1proj)[0]);
    var theta2 = Math.acos((L1 | L2)[0]);
    
    // Visualizing
    // 0x000000: black, 0xFF0000: red, // 0xCC000000: gray transparent
    return [
        0x000000, no, "theta0: " + 180 * theta0 / 3.141592, // angle in [deg]
        0x000000, "theta1: " + 180 * theta1 / 3.141592, // location TBD
        0x000000, Cbase, [up(0), elb], [endpoint, elb], endpoint,
        0xFF0000, C, target, // 
        0xFF0000, PP, elb, "theta2: " + 180 * theta2 / 3.141592,
        0xCC000000, S0, S1, P, L1proj 
    ]
    }, {
      conformal:true, gl:true, grid:true,
      useUnnaturalLineDisplayForPointPairs: true,
      width:'100%', height:'550px' // Make sure to specify a dimension.
    }));
  });
}
