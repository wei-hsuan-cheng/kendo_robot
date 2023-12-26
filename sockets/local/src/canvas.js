export function replaceCanvas(canvas) {
  let myDiv = document.getElementById("ganja-container");
  let myCanvas =
    myDiv.getElementsByTagName("canvas")[0] ||
    myDiv.getElementsByTagName("svg")[0];
  if (myCanvas == null) {
    myCanvas = document.createElement("canvas");
    myDiv.appendChild(myCanvas);
    return;
  }
  myCanvas.replaceWith(canvas);
}
