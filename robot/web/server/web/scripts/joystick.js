  function move(url) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", url, true);
      xhr.onload = function (e) {
          if (xhr.readyState === 4) {
              if (xhr.status === 200) {
                  console.log(xhr.responseText);
              } else {
                  console.error(xhr.statusText);
              }
          }
      };
      xhr.onerror = function (e) {
          console.error(xhr.statusText);
      };
      xhr.send(null);
  }

  var lspeed_slider = document.getElementById("lSpeed")
  var aspeed_slider = document.getElementById("aSpeed")

  document.querySelectorAll('.joystick a').forEach(btn => {
      btn.onclick = function () {
          var q = this.getAttribute("data-href")
          q += "&lspeed=" + lspeed_slider.value
          q += "&aspeed=" + aspeed_slider.value
          console.log(q)
          move(q);
          return false;
      }
  });

  document.addEventListener('keyup', (e) => {
      switch (e.code) {
          case 'ArrowUp':
              move('/move?direction=f')
              break;
          case 'ArrowDown':
              move('/move?direction=b')
              break;
          case 'ArrowLeft':
              move('/move?direction=l')
              break;
          case 'ArrowRight':
              move('/move?direction=r')
              break;
          default:
              move('/move?direction=s')
      }
  });
