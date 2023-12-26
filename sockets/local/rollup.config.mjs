import {nodeResolve} from "@rollup/plugin-node-resolve"; // to import node_modules packages easily
import commonjs from "@rollup/plugin-commonjs"; // convert commonjs to es6 (in case you use require)
export default {
  input: "./src/index.js",
  output: {
    file: "./build/index.bundle.js",
    format: "iife"
  },
  plugins: [
    nodeResolve(),
    commonjs({
      include: [
        "node_modules/**",
        "src/**"
      ]
    }),
  ],
  context: "this"
}