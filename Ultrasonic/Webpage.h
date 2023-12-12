const char body[] PROGMEM = R"===( 
<!DOCTYPE html>
<html>
  <body>
    <h1>Robot Mode/Direction</h1>

    <div class="slidecontainer">
      
      <p>Mode Control/Autonomous:</p>
      <input type="range" min="0" max="4" value="0" id="ModeSlider">
      <span id="ModeValue">Manual</span> <br>
      
      <p>Move Forward/Stop/Backward:</p>
      <input type="range" min="1" max="3" value="2" id="directionSlider">
      <span id="directionValue">Move Stop</span> <br>

      <p>Move Left/Stop/Right:</p>
      <input type="range" min="1" max="3" value="2" id="LRSlider">
      <span id="LRValue">Move Stop</span> <br>
      
    </div>
  </body>
  <script>
    var directionSlider = document.getElementById("directionSlider");
    var ModeSlider = document.getElementById("ModeSlider");
    var LRSlider = document.getElementById("LRSlider");

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
          document.getElementById("ModeValue").innerHTML = this.responseText;
        }
      };
      var str = "/setMode?val="; // Modify this URL to match your backend endpoint
      var res = str.concat(this.value);
      xhttp.open("GET", res, true);
      xhttp.send();
    }
    
    LRSlider.oninput = function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          document.getElementById("LRValue").innerHTML = this.responseText;
        }
      };
      var str = "/setLR?val="; // Modify this URL to match your backend endpoint
      var res = str.concat(this.value);
      xhttp.open("GET", res, true);
      xhttp.send();
    }
  </script>
</html>

)===";
