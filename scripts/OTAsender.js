var bonjour = require("bonjour")();
var process = require("process");
var http = require("http");
var fs = require("fs");
var axios = require("axios");

console.log("Searching....");
bonjour.find(
  {
    type: "http",
  },
  function (service) {
    if (service.name === "OTAUpdater" && service.txt.sys === "H2Pcs::OTA") {
      console.log("Found receiver at: ", service.addresses[0], service.port);
      var data = Buffer.from(
        fs.readFileSync(process.cwd() + "/.pio/build/uno/firmware.hex")
      ).toString("base64");
      axios
        .post(`http://${service.addresses[0]}:${service.port}/flash`, {
          data: data,
        })
        .then(() => process.exit(0))
        .catch(console.error);
    }
  }
);
