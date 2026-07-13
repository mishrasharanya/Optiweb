// Global namespace
window.AMPL_APP = window.AMPL_APP || {};

(function () {
  // Simple tab toggling for try.html handled inline;
  // This file just exposes demo samples for the Try-it pages.

  const MOVING = {
    dat: `# moving-block demo .dat (snippet)\nparam GRID_MAX := 29.;\n# ...\n`,
    mod: `# moving-block demo .mod (snippet)\n# model declarations ...\n`,
    run: `# moving-block demo .run (snippet)\noption solver ipopt; solve BLOCKTEST;`,
    out: `Ipopt 3.14.12: Optimal Solution Found\nUTOT = 123.456\n# ... more lines ...`
  };

  const FIVE = {
    dat: `# five-linked robot demo .dat (snippet)\nparam GRID_MAX := 7.;\nset SUBPHASES := 1.1 2.1;\n# ... (reft/refq/refv/refa/refu blocks) ...\n`,
    mod: `# five-linked robot demo .mod (snippet)\n# declarations ...\n`,
    run: `# five-linked robot demo .run (snippet)\noption solver ipopt; solve FIVELINKTEST;\nsolve REFSLN; solve REFOBJ;`,
    out: `Ipopt 3.14.12: Optimal Solution Found\nIPOPT cost (in local coordinates): 260.4300852955\nFMINCON cost (in local coordinates): 420.1315644238\nFMINCON cost (in ref coordinates): 469.5506519661`
  };

  window.AMPL_APP.samples = { moving: MOVING, five: FIVE };
})();
