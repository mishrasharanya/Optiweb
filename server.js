const express = require("express");
const path = require("path");

const app = express();
const PORT = 3000;

app.use(express.static(path.join(__dirname, "Public")));
app.use("/examples/neos", express.static(path.join(__dirname, "neos")));
app.use(
  "/vendor/three",
  express.static(path.join(__dirname, "node_modules", "three", "build"))
);

app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});
