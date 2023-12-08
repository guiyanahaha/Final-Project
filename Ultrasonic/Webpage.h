const char body[] PROGMEM = R"===( 
<!DOCTYPE html>
<html>
  <body>
    <h1>Motor Direction/Speed Sliders</h1>

    <div class="slidecontainer">
      
      <p>Mode Control/Autonomous:</p>
      <input type="range" min="0" max="1" value="0" id="ModeSlider">
      <span id="ModeValue">Control</span> <br>
      
      <p>Move Forward/Stop/Backward:</p>
      <input type="range" min="1" max="3" value="2" id="directionSlider">
      <span id="directionValue">Move Stop</span> <br>
      
    </div>
      <p>Turn Left/Right</p>
    <button type="button" onclick="updateLeft()"> Left </button>
      <span id="Left"> </span>
    <button type="button" onclick="updateRight()"> Right </button>
      <span id="Right"> </span> <br>
    
  </body>
  <script>
    var directionSlider = document.getElementById("directionSlider");
    var ModeSlider = document.getElementById("ModeSlider");

    directionSlider.oninput = function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("directionValue").innerHTML = this.responseText;
        }
      };
      var str = "/setDirection?val="; // Modify this URL to match your backend endpoint
      var res = str.concat(this.value);
      xhttp.open("GET", res, true);
      xhttp.send();
    }

    ModeSlider.oninput = function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("ModeValue").innerHTML = this.responseText + "%";
        }
      };
      var str = "/setMode?val="; // Modify this URL to match your backend endpoint
      var res = str.concat(this.value);
      xhttp.open("GET", res, true);
      xhttp.send();
    }
    
  updateLeft();
  function updateLeft() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("Left").innerHTML =
      this.responseText;
    }
  };
    xhttp.open("GET", "setLeft", true);
    xhttp.send();
  }
  
  updateRight();
  function updateRight() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("Right").innerHTML =
      this.responseText;
    }
  };
    xhttp.open("GET", "setLeft", true);
    xhttp.send();
  }
  </script>
</html>

)===";
