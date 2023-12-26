// Adapted from:
// https://observablehq.com/@enkimute/ganja-js-introduction
// (See very end.)

import Algebra from "ganja.js";
import { replaceCanvas } from "../util";

export function torus() {
  let canvas = Algebra(3,0,1,()=>{
    // circle is a function of two parameters, radius and bivector, that returns
    // a function of one parameter, 0<t<1, that returns a motor describing the
    // motion around a circle.
    var circle = (r,biv)=>(t)=>Math.E**(Math.PI*t*biv) * Math.E**(r*1e03);

    var c1 = circle(0.3, 1e23),
        c2 = circle(0.05,  1e13),
        c3 = circle(0.1, 1e23),         
        torus = (x,y)=>c1(x)*c2(y);   
    //return this.graph([],{gl:1})
    // rendering a 1-parameter function produces a curve
    // rendering a 2-parameter function produces a surface
    return this.graph([0x0000ff,torus,0xff0000,c3],{gl:true, width:'100%', height:'400px'})
  });
  replaceCanvas(canvas);
}