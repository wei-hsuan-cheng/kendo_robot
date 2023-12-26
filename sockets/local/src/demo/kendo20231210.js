// Adapted from:
// https://enkimute.github.io/ganja.js/examples/coffeeshop.html#dzYCK80y5

import Algebra from "ganja.js";
import { replaceCanvas } from "../util";
import { sendWebSocket } from "../webSocket";

export function kendo20231210() {
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
  
  // Defining R^3 pseudo scalar
  var I3 = e1 * e2 * e3;

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
  
  // X1 or {1}-org (trivial)
  var X1 = up(down(X0) + d1 * e3);
  var org1 = X1;
  
  ///// Target pose
  
  // Paths
  var path_sc = [];
  var path_st = [];
  
  // Initial time
  var t0 = performance.now() / 1000; // [sec]

  
  // // X6 or {6}-org (IK starting pose)
  // var X6 = up((a3 + d5) * e1 + (d4) * e2 + (d1 + a2 - d6) * e3);
  // var org6 = X6;
  // // {6}-axes (IK starting pose)
  // var x6 = e1;
  // var y6 = -e2; 
  // var z6 = -e3;
  
  // // X6 or {6}-org (Kendo sword starting pose)
  // var X6 = up((a3 + d5) * e1 + (d4) * e2 + (d1 + a2 - d6) * e3);
  // var org6 = X6;
  
  // // {6}-axes (Kendo sword starting pose)
  // var Rt1 = Math.E ** (-(Math.PI / 4) * (e1 * e3) / 2);
  // var Rt2 = Math.E ** (-(0 / 4) * ((e1 + e3) ^ (-e2)).Normalized / 2);
  // var Rt = Rt2 * Rt1;
  // var x6 = Rt * e1 * ~Rt;
  // var y6 = Rt * -e2 * ~Rt;
  // var z6 = Rt * -e3 * ~Rt;
  
  
  

  
  // Robot configuration (# 8 configurations, closed-form solution)
  var kud = 1; // elbow up: 1, elbow down: -1
  var klr = 1; // should right: 1, should left: -1 (shouler here is connected to the big arm)
  var kfn = 1; // wrist not flipped: 1, wrist flipped: -1
  
  // frame axes lengths
  var axes_legnth = 0.1;

  ////////// Graph the items //////////
  // var robot = document.body.appendChild(this.graph(() => {
  var robot = replaceCanvas(this.graph(() => {
      // Sword swinging speed (period)
      var t_swing = 0.7; // [s]
      
      // Sword dimensions and offsets
      var l_sword = 1200 * mm2m; // sword length
      var offset_sword = 100 * mm2m; // sword offset in z direction
      var R6sc = 1;
      var Rscst = 1; // sword centre of rotation and tip are of same orientation (rigid body)
      var M6sc = (1 - 0.5 * offset_sword * (-e3 ^ ni)); // X6 -> sword centre of rotation
      var Mscst = (1 - 0.5 * l_sword * (e1 ^ ni)); // sword centre of rotation -> sword tip
      var M6st = M6sc * Mscst; // X6 -> sword tip
      
      //////////////////////////////////////////////////////////////////////
      ////////// Sword initial and final position //////////////////////////
      //////////////////////////////////////////////////////////////////////
      
      // Sword tip target position (depends on human pose)
      var Xst_target = up( (a3 + d5) * 3.5 * 0.89 * e1 + (d4)* 1.5 * e2 + (d1 + a2 - d6) * 2 * 0.98 * e3 );
      
      
      var P_noX1st = (no ^ e3 ^ Xst_target ^ ni).Normalized;
      var P_noe3e1 = (no ^ e3 ^ e1 ^ ni).Normalized;
      var ang_Xste1 = Math.acos((P_noX1st | P_noe3e1)[0]); // angle between tip target position and x-axis
      
      var L_X1Xst =  (X1 ^ Xst_target ^ ni).Normalized;
      var L_X1Xst_proj = ( (L_X1Xst | (X1 ^ e1 ^ e2 ^ ni)) | (X1 ^ e1 ^ e2 ^ ni) ).Normalized;
      var ang_Xste1e2 = Math.acos((L_X1Xst | L_X1Xst_proj)[0]); 
      ang_Xste1e2 = Math.PI - ang_Xste1e2;
      // If need to restrict angle quadrants
      // if (ang_Xste1e2 > Math.PI && ang_Xste1e2 < 2 * Math.PI) {
      // ang_Xste1e2 = ang_Xste1e2 - 2 * Math.PI; ang_Xste1e2 = Math.PI - ang_Xste1e2;
      // } else {
      // ang_Xste1e2 = Math.PI - ang_Xste1e2;
      // }
      
      var Rz_ang_Xste1 = Math.E ** (-ang_Xste1 * (e1 * e2) / 2);
      
      // Sword rotation centre initial position
      var Xsc_i = up( down(X1) + 0.25 * (down(Xst_target) - down(X1)) );
      var T0sc_i = 1 - 0.5 * (down(Xsc_i) ^ ni);
      
      // Sword tip initial position
      var Trans_dir_st = (down(Xst_target) - down(X1)).Grade(1).Normalized;
      var Xst_i = up( down(Xsc_i) + l_sword * Trans_dir_st );
      var T0st_i = 1 - 0.5 * (down(Xst_i) ^ ni);
      
      // Sword rotation centre final position
      var ang_st_f = 20 * d2r;
      var dang_sc_if = -ang_Xste1e2 + ang_st_f; // 20 [deg] contact angle w/ the opponent
      var Rsc_if = Math.E ** (-dang_sc_if * (Trans_dir_st ^ e3) / 2); 
      var Xsc_f = up( down(Xst_target) + l_sword * Rsc_if * (-Trans_dir_st) * ~Rsc_if );
      var T0sc_f = 1 - 0.5 * (down(Xsc_f) ^ ni);
      
      // Sword rotation centre translation direction
      var Trans_dir_sc = (down(Xsc_f) - down(Xsc_i)).Normalized; // directional vector
      var Trans_dis_sc = down(Xsc_f) - down(Xsc_i); Trans_dis_sc = Math.sqrt((Trans_dis_sc * Trans_dis_sc)[0]); // distance
      
      // Sword rotation centre initial orientation
      var xsc_i = Trans_dir_st;
      var ysc_i = (xsc_i ^ e3) / I3;
      var zsc_i = (xsc_i ^ ysc_i) / I3;
      var phi0sc_i = 1 + e1 * xsc_i + e2 * ysc_i + e3 * zsc_i;
      var R0sc_i = phi0sc_i.Normalized; // Same result as "var R0sc_i = phi0sc_i / phisc_i.Length", and "var R0sc_i = phi0sc_i / Math.sqrt((phisc_i * ~phisc_i)[0])";
      
      // Sword tip initial orientation
      var R0st_i = R0sc_i;
      
      //////////////////////////////////////////////////////////////////////
      ////////// Synthesize trajectory based on the given positions ////////
      //////////////////////////////////////////////////////////////////////
      
      // Time
      var t_elapse = (performance.now() / 1000) - t0;
      var t = t_elapse % t_swing;
      
      
      ////////// Trajectory planning using cubic splines //////////
      // initial -> ang_Xste1e2
      // maximum -> ang_max_reach
      // final -> ang_st_f
      var ang_max_reach = Math.PI / 2; // [rad]
      const dt = t_swing / 2;
      var t_tilde;
      
      var x0 =       down(Xsc_i); var x2 =       down(Xsc_f); var x1 = (x0 + x2) / 2;
      var dotx0 =              0; var dotx2 =              0; var dotx1 = 1.5*(x2 - x0) / t_swing;
      var dx0 =          x1 - x0; var dx1 =          x2 - x1;
      var sdotx0 = dotx1 + dotx0; var sdotx1 = dotx2 + dotx1;
      
      var dxsc; var dangs; 
      var kappa0; var nu0;
      var kappa1; var nu1;
      var kappa2; var nu2;
      var kappa3; var nu3;
      var xsc; var Xsc; var angs_cubic; var dR0sc;
      
      if (t >= 0 && t < t_swing / 2) {
        t_tilde = t - 0;
        // Translation
        kappa0 = x0;
        kappa1 = dotx0;
        kappa2 = (3 / (dt**2)) * dx0 - (2 / dt) * dotx0 - (1 / dt) * dotx1; 
        kappa3 = (-2 / (dt**3)) * dx0 + (1 / (dt**2)) * sdotx0;
        // Rotation
        dangs = ang_max_reach - ang_Xste1e2;
        nu0 = ang_Xste1e2; 
      } 
      else {
        t_tilde = t - t_swing / 2;
        // Translation
        kappa0 = x1;
        kappa1 = dotx1;
        kappa2 = (3 / (dt**2)) * dx1 - (2 / dt) * dotx1 - (1 / dt) * dotx2; 
        kappa3 = (-2 / (dt**3)) * dx1 + (1 / (dt**2)) * sdotx1;
        // Rotation
        dangs = ang_st_f - ang_max_reach;
        nu0 = ang_max_reach; 
      }
      
      xsc = ( kappa0 + kappa1 * t_tilde + kappa2 * (t_tilde**2) + kappa3 * (t_tilde**3) ).Grade(1);
      Xsc = up(xsc);

      nu1 = 0; 
      nu2 = (3 / (dt**2)) * dangs; 
      nu3 = (-2 / (dt**3)) * dangs; 
      angs_cubic = nu0 + nu1 * t_tilde + nu2 * (t_tilde**2) + nu3 * (t_tilde**3);
      dR0sc = Math.E ** ( -(angs_cubic - ang_Xste1e2) * (Trans_dir_st ^ e3) / 2 );
      
    

      ////////// Trajectory planning (periodic) //////////
      // Sword rotation centre trajectory- position
      // Xsc = up( down(Xsc_i) + (down(Xsc_f) - down(Xsc_i)) * 0.5 * (Math.sin(2 * Math.PI * t / t_swing) + 1) );

      // Sword rotation centre trajectory- orientation
      // dR0sc = Math.E ** ( -ang_max_reach * ( 0.5 * (Math.sin(2 * Math.PI * t / t_swing) + 1) ) * (Trans_dir_st ^ e3) / 2);


      var orgsc = Xsc;
      var T0sc = 1 - 0.5 * (down(Xsc) ^ ni);
      
      var R0sc = R0sc_i * dR0sc;
      var xsc = dR0sc * xsc_i * ~dR0sc;
      var ysc = dR0sc * ysc_i * ~dR0sc;
      var zsc = dR0sc * zsc_i * ~dR0sc;
      
      // Sword rotation centre trajectory- pose
      var M0sc = T0sc * R0sc;
      
      // Sword tip trajectory- pose
      var Xst = up(down(Xsc) + l_sword * xsc);
      var orgst = Xst;
      var R0st = R0sc * Rscst;
      var xst = Rscst * xsc * ~Rscst;
      var yst = Rscst * ysc * ~Rscst;
      var zst = Rscst * zsc * ~Rscst;
      
      // End-effector trajectory
      // {6}-axes (end-effector orientation)
      var Rsc6 = 1 / R6sc;
      var x6 = Rsc6 * xsc * ~Rsc6;
      var y6 = Rsc6 * ysc * ~Rsc6;
      var z6 = Rsc6 * zsc * ~Rsc6;
      // X6 or {6}-org (end-effector position)
      var X6 = up(down(Xsc) - offset_sword * zsc);
      var org6 = X6;
      
    
      //////////////////////////////////////////////////////////////////////
      ////////// Inverse kinematics of the robot ///////////////////////////
      //////////////////////////////////////////////////////////////////////
      
      ////////// X5 //////////
      var X5 = up(down(X6) - d6 * z6); 
      var org5 = X5;
    
      
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

      // Add to the path_x array and pop
      path_sc.push(Xsc);
      if (path_sc.length > 100) {
          path_sc.shift();
      }
      path_st.push(Xst);
      if (path_st.length > 100) {
          path_st.shift();
      }
      
      
      sendWebSocket(`${t}`, 10);
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
          0xCC0000FF, Xst_target,
          0xCC008000, Xsc_i, 
          0xCC008000, Xst_i, 
          0xCC0000FF, Xsc_f,
          0xCC008000, [Xsc_i, Xst_i],
          0xCC0000FF, [Xsc_f, Xst_target], 
          
          // Base circle
          0x00000000, C0, 
          
          // Time
          0xFF0000, "t=" + parseFloat(t).toFixed(2) + "[s]",
          
          
          ////////// Quick demo //////////
          
            ///// Frames based on D-H parameters
            // Robot frames
            0xFF0000, [org0, up(down(org0) + axes_legnth * x0)], // {0} (RGB - e1e2e3)
            0x008000, [org0, up(down(org0) + axes_legnth * y0)],
            0x0000FF, [org0, up(down(org0) + axes_legnth * z0)], 
            
            // 0xFF0000, [org6, up(down(org6) + axes_legnth * x6)], // {6}
            // 0x008000, [org6, up(down(org6) + axes_legnth * y6)],
            // 0x0000FF, [org6, up(down(org6) + axes_legnth * z6)],
            
            0xFF0000, [orgsc, up(down(orgsc) + axes_legnth * xsc)], // {sc}
            0x008000, [orgsc, up(down(orgsc) + axes_legnth * ysc)],
            0x0000FF, [orgsc, up(down(orgsc) + axes_legnth * zsc)], 
            
            0xFF0000, [orgst, up(down(orgst) + axes_legnth * xst)], // {st}
            0x008000, [orgst, up(down(orgst) + axes_legnth * yst)],
            0x0000FF, [orgst, up(down(orgst) + axes_legnth * zst)],
            
            // ///// Null points for IK problem
            // Endpoints
            0xFF0000, endpoint5,
            0xFF0000, endpoint4,
            0xFF0000, endpoint3,
            0xFF0000, endpoint2, "" + unreachable,
            
            // Null points
              0xFF0000, Xsc,
              0xFF0000, Xst,
            0x00000000, X0, //"theta1=" + parseFloat(theta1 * (180 / Math.PI)).toFixed(3) + " [deg]",
            0xCC000000, X6, //"theta6=" + parseFloat(theta6 * (180 / Math.PI)).toFixed(3) + " [deg]",
            0x00000000, X5, //"theta5=" + parseFloat(theta5 * (180 / Math.PI)).toFixed(3) + " [deg]",
            0x00000000, X1, 
            0x00000000, X4, //"theta4=" + parseFloat(theta4 * (180 / Math.PI)).toFixed(3) + " [deg]",
            0x00000000, X3, //"theta3=" + parseFloat(theta3 * (180 / Math.PI)).toFixed(3) + " [deg]",
            0x00000000, X2, //"theta2=" + parseFloat(theta2 * (180 / Math.PI)).toFixed(3) + " [deg]",
  
            
            // Null points links
            0x000000, [X0, X1], 
            0xCC000000, [Xsc, X6],
            0xFF0000, [Xst, Xsc],
            0x000000, [X5, X6],
            0x000000, [X4, X5],
            0x000000, [X3, X4],
            0x000000, [X2, X3],
            0x000000, [X1, X2],
            
            // Lines
            // 0xCC000000, L54,
            // 0xCC000000, L34,
            // 0xCC000000, L23,
            // 0xCC000000, L12,
            // 0xCC000000, L01,
          
          ////////// Detailed demo following the procedure //////////
          
          //
          
          ////////// Play around w/ all the entities //////////
          
            // ///// Frames based on D-H parameters
            // // Robot frames
            // 0xFF0000, [org0, up(down(org0) + axes_legnth * x0)], // {0} (RGB - e1e2e3)
            // 0x008000, [org0, up(down(org0) + axes_legnth * y0)],
            // 0x0000FF, [org0, up(down(org0) + axes_legnth * z0)],
            
            // 0xFF0000, [org6, up(down(org6) + axes_legnth * x6)], // {6}
            // 0x008000, [org6, up(down(org6) + axes_legnth * y6)],
            // 0x0000FF, [org6, up(down(org6) + axes_legnth * z6)],
            
            // 0xFF0000, [orgtip, up(down(orgtip) + axes_legnth * xtip)], // sword tip
            // 0x008000, [orgtip, up(down(orgtip) + axes_legnth * ytip)],
            // 0x0000FF, [orgtip, up(down(orgtip) + axes_legnth * ztip)],
            
            // // Robot links
            // 0x00000000, [org0, org1], // {0} -> {1}
            // 0x00000000, [org5, org6], // {5} -> {6}
            
            // // Frame origins
            // 0x00000000, org0, // "{0}",
            // 0x00000000, org6, // "{6}", "org6 = " + org6,
            
            ///// Null points for IK problem
            
            // // Endpoints
            // 0xFF0000, endpoint5,
            // 0xFF0000, endpoint4,
            // 0xFF0000, endpoint3,
            // 0xFF0000, endpoint2,
            
            // // Null points
            // 0x00000000, X0, //"theta1=" + parseFloat(theta1 * (180 / Math.PI)).toFixed(3) + " [deg]",
            //   0xFF0000, X6, //"theta6=" + parseFloat(theta6 * (180 / Math.PI)).toFixed(3) + " [deg]",
            //   0xFF0000, Xtip, 
            // 0x00000000, X5, //"theta5=" + parseFloat(theta5 * (180 / Math.PI)).toFixed(3) + " [deg]",
            // 0x00000000, X1, 
            // 0x00000000, X4, //"theta4=" + parseFloat(theta4 * (180 / Math.PI)).toFixed(3) + " [deg]",
            // 0x00000000, X3, //"theta3=" + parseFloat(theta3 * (180 / Math.PI)).toFixed(3) + " [deg]",
            // 0x00000000, X2, //"theta2=" + parseFloat(theta2 * (180 / Math.PI)).toFixed(3) + " [deg]",

            // // Auxiliary null points
            // 0x0000FF, Xc,
            
            
            // // Null points links
            // 0x000000, [X0, X1],
            // 0xFF0000, [Xtip, X6],
            // 0x000000, [X5, X6],
            // 0x000000, [X4, X5],
            // 0x000000, [X3, X4],
            // 0x000000, [X2, X3],
            // 0x000000, [X1, X2],
            
            // Auxiliary null point links
            // 0x0000FF, [X5, Xc],
            
            
            // ///// Geometric entities
            // Spheres, planes, lines, and point pairs
            // // Point pairs, circles, lines, planes, and spheres
            
            
            // // Point pairs
            // 0x0000FF, PPc,
            // 0x0000FF, PP4,
            // 0x0000FF, PP3,
            // 0x0000FF, PP2, 
            
            // // Circles
            // 0xCC0000FF, C5k,
            // 0xCC0000FF, C2,
            
            // // Spheres
            // 0xCCFF0000, Sc, //"" + Sc, 
            // 0xCC0000FF, K0, //"" + K0,
            // 0xCCFF0000, S5, //"" + S5,
            // 0xCC0000FF, S4, //"" + S4,
            // 0xCC0000FF, S3, //"" + S4,
            // 0xCCFF0000, S1, //"" + S4, 
          
            
            
            // // Lines
            // 0xCC000000, L54,
            // 0xCC000000, L34,
            // 0xCC000000, L23,
            // 0xCC000000, L12,
            // 0xCC000000, L01,
            
            // // Planes
            // 0xCC008000, (X5 ^ e1 ^ e2 ^ ni),
            // 0xCC008000, Pc,
            // 0xCC0000FF, Pc_ver,
            // 0xCC0000FF, P56,
            // 0xCC0000FF, Pc_hor,
            

          0x550000FF].concat( [...Array(path_sc.length - 1).keys()].map(i => [path_sc[i], path_sc[i + 1]]),
                              [...Array(path_st.length - 1).keys()].map(i => [path_st[i], path_st[i + 1]]),
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
      h: 50 * 1 * d2r,  // e12 plane
      p: -22.5 * 1 * d2r,// e23 plane
      scale: 1, // custom scale (default = 1), mousewheel.

  }));

});
}
