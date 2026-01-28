const express = require("express");
const bodyParser = require("body-parser");
const fs = require("fs");
const NEOS = require("neos-js");

const app = express();
const PORT = 3000;

app.use(express.static("public"));
app.use(bodyParser.json());

//generate .dat file route
app.post('/generate', (req, res) => {
  const { fileContent } = req.body;

  // e.g., write it to test.dat
  fs.writeFileSync('test.dat', fileContent);
  res.send('.dat file generated and saved successfully!');
});

app.post("/generate-mod", (req, res) => {
  const { fileContent } = req.body;
  try {
    fs.writeFileSync("test.mod", fileContent);
    res.send("test.mod saved successfully!");
  } catch (err) {
    console.error("MOD Save Error:", err);
    res.status(500).send("Error saving test.mod");
  }
});

app.post("/generate-run", (req, res) => {
  const { fileContent } = req.body;
  try {
    fs.writeFileSync("test.run", fileContent);
    res.send("test.run saved successfully!");
  } catch (err) {
    console.error("RUN Save Error:", err);
    res.status(500).send(" Error saving test.run");
  }
});


// === NEOS SUBMIT ROUTE ===
app.post("/submit-neos", async (req, res) => {
  try {
    const solver = req.body.solver || "IPOPT";

    // Check required files exist
    if (!fs.existsSync("test.mod") || !fs.existsSync("test.dat") || !fs.existsSync("test.run")) {
      return res.status(400).send("Missing one or more required files: test.mod, test.dat, or test.run.");
    }

    // Read the contents of the files
    const model = fs.readFileSync("test.mod", "utf8");
    const data = fs.readFileSync("test.dat", "utf8");
    const commands = fs.readFileSync("test.run", "utf8");

    // Build XML string for NEOS
    const xml = await NEOS.xmlstring({
      category: "nco",
      solver,
      inputMethod: "AMPL",
      model,
      data,
      commands,
      email: "sharanyamishra8@gmail.com"
    });

    // Submit to NEOS
    const job = await NEOS.submitJob(xml);

    // Job confirmation
    const msg = `
NEOS job submitted successfully!

Job Number: ${job.jobNumber}
Password: ${job.password}

Results will be sent to your email (sharanyamishra8@gmail.com).

You can also manually track it here:
https://neos-server.org/neos/admin.html
`;

    console.log(msg); // Log in terminal
    res.send(msg);    // Send to frontend

  } catch (err) {
    console.error("NEOS Submission Error:", err);
    res.status(500).send("Error submitting job to NEOS.");
  }
});

// === START SERVER ===
app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});
