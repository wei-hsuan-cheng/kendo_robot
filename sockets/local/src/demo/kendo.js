// Adapted from:
// https://enkimute.github.io/ganja.js/examples/coffeeshop.html#dzYCK80y5

import Algebra from "ganja.js";
import { replaceCanvas } from "../util";

export function kendo() {
  // This framework is adapted from: https://slides.com/hugohadfield/game2020
  // Operator overloading .. * = geometric product, | = inner, ^ = wedge, & = vee, << = dot, >>> = sandwich ...
  // References for this code:
  // https://slides.com/hugohadfield/game2020
  // https://www.mic-journal.no/ABS/MIC-2016-1-6.asp/
  // https://www.sciencedirect.com/science/article/pii/S0094114X22001045

  // Operator overloading .. * = geometric product, ^ = wedge, & = vee, << = dot, >>> = sandwich, ! = dual, 

  // Create a Clifford Algebra with 4,1 metric for 3D CGA. 
  Algebra(4, 1, () => {

    // For right-hand rule, and a better angle of view (e3 toward up)
    var e1 = 1e1;
    var e2 = 1e3;
    var e3 = 1e2;

    // Defining a null basis, and upcasting for points
    var ni = 1e4 + 1e5, no = 0.5e5 - 0.5e4;
    var up = (x) => no + x + 0.5 * x * x * ni;
    var down = (x) => (x ^ (no ^ ni)) * (no ^ ni) / (-x | ni);
    
    // Unit trasforms
    var r2d = 180 / Math.PI; // [rad] to [deg]
    var d2r = 1 / r2d; // [deg] to [rad]
    var mm2m = 1 / 1000;
    var m2mm = 1 / mm2m;

    // Set up robot D-H parameters (angle in [rad], length in [m])
    // alpha_{i-1}, a_{i-1}, d_{i}, theta_{i} (theta_{i}s are defined in the next section)
    var alpha0 = 0; var a0 = 0; var d1 = 165.2 * mm2m; // frame {0} -> {1}
    var alpha1 = -Math.PI / 2; var a1 = 0; var d2 = 0; // frame {1} -> {2}
    var alpha2 = 0; var a2 = 536.1 * mm2m; var d3 = 0; // frame {2} -> {3}
    var alpha3 = 0; var a3 = 457.9 * mm2m; var d4 = -156.3 * mm2m; // frame {3} -> {4}
    var alpha4 = -Math.PI / 2; var a4 = 0; var d5 = 106 * mm2m; // frame {4} -> {5}
    var alpha5 = -Math.PI / 2; var a5 = 0; var d6 = 113.15 * mm2m; // frame {5} -> {6}

    // Draw the robot base as a grade-3 circle
    var base_circle_length = a2 + a3;
    var C0 = (up(base_circle_length * e1) ^ up(base_circle_length * e2) ^ up(-base_circle_length * e1)).Normalized; // base circle
    
    // X0 or {0}-org
    var X0 = no;
    var org0 = X0;
    // {0}-axes
    var x0 = e1; 
    var y0 = e2; 
    var z0 = e3;
    
    ///// Target pose
    
    // Paths
    var path_tip = [];
    var path_6 = [];
  
    
    // Robot configuration (# 8 configurations, closed-form solution)
    var kud = 1; // elbow up: 1, elbow down: -1
    var klr = 1; // should right: 1, should left: -1 (shouler here is connected to the big arm)
    var kfn = 1; // wrist not flipped: 1, wrist flipped: -1
    
    // frame axes lengths
    var axes_legnth = 0.1;

    ////////// Graph the items.  
    // var robot = document.body.appendChild(this.graph(() => {
    var robot = replaceCanvas(this.graph(() => {
    // Kendo sword swinging speed
    var speed = 10 * 0.5;
    
    // Kendo sword tip pose
    var l_sword = 1200 * mm2m; // sword length
    var X6_i = up((a3 + d5) * e1 + (d4) * e2 + (d1 + a2 - d6) * e3); // sword tip centre initial position
    var T06_i = 1 - 0.5 * (X6_i ^ ni);
    var Rx180 = Math.E ** (-Math.PI * (e2 * e3) / 2); var Ry90 = Math.E ** (-(Math.PI / 2) * (e3 * e1) / 2);
    var R06_i = Math.E ** (-(70) * d2r * (e1 * e3) / 2); // sword tip initial orientation
    var T6tip = 1 - 0.5 * l_sword * (e1 ^ ni); // offset transform from end-effector to sword tip
    
    var dR = Math.E ** (-(-50 * 0.5 * (Math.sin(speed * performance.now() / 1000) + 1)) * d2r * (e1 * e3) / 2);
    var dT = 1 - 0.5 * (l_sword * 1 * 0.5 * (Math.sin(speed * performance.now() / 1000) + 1)) * (e1 ^ ni);
    
    var R06 = R06_i * dR;
    var R0tip = R06;
    
    var M06 = T06_i * dT * dR; // end-effector pose time function
    
    
    // X6 or {6}-org (end-effector position)
    var X6 = M06 * no * ~M06;
    var org6 = X6;
    // {6}-axes (end-effector orientation)
    var x6 = R06 *  e1 * ~R06;
    var y6 = R06 * -e2 * ~R06;
    var z6 = R06 * -e3 * ~R06;
    
    // Xtip
    var Xtip = up(down(X6) + l_sword * x6);
    var orgtip = Xtip;
    // tip-axes
    var xtip = x6;
    var ytip = y6;
    var ztip = z6;
    
    ////////// The actual inverse kinematics of the robot //////////
    ////////// X5 and X1 (trivial) //////////
    var X5 = up(down(X6) - d6 * z6); 
    var org5 = X5;
    
    var X1 = up(down(X0) + d1 * e3);
    var org1 = X1;
    
    ////////// X3 and X4 //////////
    // Intersect 2 spheres gives 1 circle
    var Sc = !(X5 - 0.5 * (d4 ** 2) * ni); // grade-4 sphere
    var K0 = (!(no - (!(Sc) | no) * ni)).Grade(4).Normalized; // grade-4 sphere
    var C5k = (Sc & K0).Normalized; // grade-3 circle
    // Intersect the circle and a plane gives a point pair
    var PPc = -((X5 ^ e1 ^ e2 ^ ni) & C5k).Grade(2).Normalized; // grade-2 point pair
    var PPcd = (PPc | PPc) / ((PPc ^ ni) | (PPc ^ ni)); // point pair distance
    
    // The square of the point pair describes if the spheres intersect
    if ((PPc * PPc)[0] > 0){
    // If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    var Xc = (1 + klr * PPc * (1 / Math.sqrt((PPc * PPc)[0]))) * (PPc | ni); Xc = up(down(Xc));
    var endpoint5 = X5;
    }
    else{
    // If the sphere at origin vanishes (radius = 0), the point pair becomes a single point
    var endpoint5 = X5;
    var Xc = no;
    }
    
    // Vertical plane passing through Xc
    var Pc = no ^ e3 ^ Xc ^ ni;
    
    // Pc shift to pass through X5
    var Pc_ver = !(!(Pc) + (X5 | !(Pc)) * ni);
    
    // Plane perp to Pc_ver and pass through X5
    var P56 = !(X5 ^ X6) ^ ni;
    var n56 = - ((P56 | no) | ni).Normalized;
    var Pc_hor = X5 ^ n56 ^ ni;
    
    // Line passing through X4 and X5
    var L54 = Pc_ver & Pc_hor;
    
    // Sphere centred at X5 w/ radius d5
    var S5 = (!(X5 - 0.5 * (d5 ** 2) * ni)).Grade(4).Normalized;
    
    // Intersect the sphere and a line gives a point pair
    var PP4 = !(L54) | S5;
    var PP4d = (PP4 | PP4) / ((PP4 ^ ni) | (PP4 ^ ni)); // point pair distance
    
    if ((PP4 * PP4)[0] > 0){
    // If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    var X4 = (1 + kfn * PP4 * (1 / Math.sqrt((PP4 * PP4)[0]))) * (PP4 | ni); X4 = up(down(X4));
    var endpoint4 = X4;
    }


    // Sphere centred at X4 w/ radius d4
    var S4 = (!(X4 - 0.5 * (d4 ** 2) * ni)).Grade(4).Normalized;
    var L34 = X4 ^ !(Pc) ^ ni;
    
    // Intersect the sphere and a line gives a point pair
    var PP3 = !(L34) | S4;
    var PP3d = (PP3 | PP3) / ((PP3 ^ ni) | (PP3 ^ ni)); // point pair distance
    
    if ((PP3 * PP3)[0] > 0){
    // If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    var X3 = (1 - klr * PP3 * (1 / Math.sqrt((PP3 * PP3)[0]))) * (PP3 | ni); X3 = up(down(X3));
    var endpoint3 = X3;
    }

    
    ////////// X2 //////////
    // Spheres centred at X1 w/ radius a2 and centred at X3 w/ radius a3
    var S3 = (!(X3 - 0.5 * (a3 ** 2) * ni)).Grade(4).Normalized; // grade-4 plane
    var S1 = (!(X1 - 0.5 * (a2 ** 2) * ni)).Grade(4).Normalized; // grade-4 plane
    var C2 = (S1 & S3).Normalized; // grade-3 circle
    
    // Intersect the circle and a plane gives a point pair
    var PP2 = -(Pc & C2).Grade(2).Normalized; // grade-2 point pair
    var PP2d = (PP2 | PP2) / ((PP2 ^ ni) | (PP2 ^ ni)); // point pair distance
    
    if ((PP2 * PP2)[0] > 0){
    // If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
    var X2 = (1 + kud * PP2 * (1 / Math.sqrt((PP2 * PP2)[0]))) * (PP2 | ni); X2 = up(down(X2));
    var endpoint2 = X2;
    var unreachable = [];
    }
    else{
    // If the spheres do not intersect then we will just reach for the object.
    var endpoint2 = up(down(X1) + (a2 + a3) * (down(X3)- down(X1)).Normalized);
    var X2 = up(down(X1) + a2 * (down(X3)- down(X1)).Normalized);
    var unreachable = "unreachable!";
    }
    
    ////////// Solve for IK position problem //////////
    // Form lines
    var L01 = (no ^ e3 ^ ni).Normalized; // grade-3 line
    var L12 = (X1 ^ X2 ^ ni).Normalized; // grade-3 line
    var L23 = (X2 ^ X3 ^ ni).Normalized; // grade-3 line

    ////////// Solve for joint variables //////////
    // ath_{i}, bth_{i}, Nth_{i}, offset_{i} [rad]
    var ath1 = e2; ath1 = ath1.Grade(1).Normalized;
    var bth1 = klr * -(!Pc); bth1 = bth1.Grade(1).Normalized;
    var Nth1 = e1 ^ e2; Nth1 = Nth1.Grade(2).Normalized;
    var offset1 = 0;
    
    var ath2 = L01 | (ni ^ no); ath2 = ath2.Grade(1).Normalized;
    var bth2 = L12 | (ni ^ no); bth2 = bth2.Grade(1).Normalized;
    var Nth2 = klr * (Pc | no) | ni; Nth2 = Nth2.Grade(2).Normalized;
    var offset2 = -Math.PI / 2;
    
    var ath3 = L12 | (ni ^ no); ath3 = ath3.Grade(1).Normalized;
    var bth3 = L23 | (ni ^ no); bth3 = bth3.Grade(1).Normalized;
    var Nth3 = klr * (Pc | no) | ni; Nth3 = Nth3.Grade(2).Normalized;
    var offset3 = 0;
    
    var ath4 = L23 | (ni ^ no); ath4 = ath4.Grade(1).Normalized;
    var bth4 = -(L54 | (ni ^ no)); bth4 = bth4.Grade(1).Normalized;
    var Nth4 = klr * (Pc | no) | ni; Nth4 = Nth4.Grade(2).Normalized;
    var offset4 = - Math.PI / 2;
    
    var ath5 = klr * -(!Pc); ath5 = ath5.Grade(1).Normalized;
    var bth5 = -z6; bth5 = bth5.Grade(1).Normalized;
    var Nth5 = (-(!L54) ^ no) | ni; Nth5 = Nth5.Grade(2).Normalized; 
    var offset5 = 0;
    
    var ath6 = -(L54 | (ni ^ no)); ath6 = ath6.Grade(1).Normalized;
    var bth6 = -y6; bth6 = bth6.Grade(1).Normalized;
    var Nth6 = -n56; Nth6 = Nth6.Grade(2).Normalized;
    var offset6 = 0;
    
    // Solving the joint variables
    // THE JOINT VARIABLES ARE ONLY VALID FOR SHOULDER RIGHT CONGI SO FAR
    var theta1 = Math.atan2(((ath1 ^ bth1) * (Nth1 ** -1))[0], (ath1 | bth1)[0]) + offset1;
    var theta2 = Math.atan2(((ath2 ^ bth2) * (Nth2 ** -1))[0], (ath2 | bth2)[0]) + offset2;
    var theta3 = Math.atan2(((ath3 ^ bth3) * (Nth3 ** -1))[0], ath3 | bth3) + offset3;
    var theta4 = Math.atan2(((ath4 ^ bth4) * (Nth4 ** -1))[0], ath4 | bth4) + offset4;
    var theta5 = Math.atan2(((ath5 ^ bth5) * (Nth5 ** -1))[0], ath5 | bth5) + offset5;
    var theta6 = Math.atan2(((ath6 ^ bth6) * (Nth6 ** -1))[0], ath6 | bth6) + offset6;

    // Add to the path_tip array and pop
    path_tip.push(Xtip);
    if (path_tip.length > 100) {
        path_tip.shift();
    }
    path_6.push(X6);
    if (path_6.length > 100) {
        path_6.shift();
    }
        
    
    ////////// Visualising //////////
    // HTML CSS color
    // 0x000000: black, 0xCC000000: black transparent,
    // 0xFF0000: red, 0xCCFF0000: red transparent,
    // 0x008000: green, 0xCC008000 : green transparent
    // 0x0000FF: blue, 0xCC0000FF: blue transparent,
    // Detailed color explanation: https://majerle.eu/documentation/gui/html/group___g_u_i___c_o_l_o_r_s.html#ga5dbf333a6f39a1d015414539722c5d8f

    return [
      // Testing
      // 0x00000000, up(-(!Pc)), 
      // 0xFF0000, up(L23 | (no ^ ni)),
      // 0x008000, up(-(L54 | (ni ^ no))),
      // 0x00000000, up(-y6),
      
      // Base circle
      0x00000000, C0,
      
      ////////// Quick demo //////////
      
      ///// Frames based on D-H parameters
      // Robot frames
      0xFF0000, [org0, up(down(org0) + axes_legnth * x0)], // {0} (RGB - e1e2e3)
      0x008000, [org0, up(down(org0) + axes_legnth * y0)],
      0x0000FF, [org0, up(down(org0) + axes_legnth * z0)],
      
      0xFF0000, [org6, up(down(org6) + axes_legnth * x6)], // {6}
      0x008000, [org6, up(down(org6) + axes_legnth * y6)],
      0x0000FF, [org6, up(down(org6) + axes_legnth * z6)],
      
      0xFF0000, [orgtip, up(down(orgtip) + axes_legnth * xtip)], // sword tip
      0x008000, [orgtip, up(down(orgtip) + axes_legnth * ytip)],
      0x0000FF, [orgtip, up(down(orgtip) + axes_legnth * ztip)],
      
      // ///// Null points for IK problem
      // Endpoints
      0xFF0000, endpoint5,
      0xFF0000, endpoint4,
      0xFF0000, endpoint3,
      0xFF0000, endpoint2, "" + unreachable,
      
      // Null points
      0x00000000, X0, //"theta1=" + parseFloat(theta1 * (180 / Math.PI)).toFixed(3) + " [deg]",
        0xFF0000, X6, //"theta6=" + parseFloat(theta6 * (180 / Math.PI)).toFixed(3) + " [deg]",
        0xFF0000, Xtip, 
      0x00000000, X5, //"theta5=" + parseFloat(theta5 * (180 / Math.PI)).toFixed(3) + " [deg]",
      0x00000000, X1, 
      0x00000000, X4, //"theta4=" + parseFloat(theta4 * (180 / Math.PI)).toFixed(3) + " [deg]",
      0x00000000, X3, //"theta3=" + parseFloat(theta3 * (180 / Math.PI)).toFixed(3) + " [deg]",
      0x00000000, X2, //"theta2=" + parseFloat(theta2 * (180 / Math.PI)).toFixed(3) + " [deg]",
      
      
      // Null points links
      0x000000, [X0, X1],
      0xFF0000, [Xtip, X6],
      0x000000, [X5, X6],
      0x000000, [X4, X5],
      0x000000, [X3, X4],
      0x000000, [X2, X3],
      0x000000, [X1, X2],
      
      // Lines
      0xCC000000, L54,
      0xCC000000, L34,
      0xCC000000, L23,
      0xCC000000, L12,
      0xCC000000, L01,
    
      0x550000FF]
      .concat(
        [...Array(path_tip.length - 1).keys()].map(i => [path_tip[i], path_tip[i + 1]]),
        [...Array(path_6.length - 1).keys()].map(i => [path_6[i], path_6[i + 1]])
      )    
    }, {
        // Display settings
        conformal: true, gl: true, animate: true,
        useUnnaturalLineDisplayForPointPairs: true, // for CGA
        width:'100%', height:'400px',

        // Grids and labels
        grid: true, // display a grid
        // labels: true, // label the grid

        // Shapes and texts
        pointRadius: 1, // custon point radius (default = 1)
        lineWidth: 2, // custom lineWidth (default = 1)
        // fontSize: 1, // custom font size (default = 1)

        // viewing angles and scales
        h: 60 * d2r,  // e12 plane
        p: -22.5 * d2r,// e23 plane
        scale: 1, // custom scale (default = 1), mousewheel.
    }));
  });
}
