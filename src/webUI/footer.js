(() => {
  const el = document.getElementById("fwFooter");
  if (!el) return;

  fetch("/info.json", { cache: "no-store" })
    .then((r) => (r.ok ? r.json() : null))
    .then((j) => {
      if (!j || !j.version) return;
      el.textContent = "Firmware v" + j.version + " - BattBrrr Controller";
    })
    .catch(() => {});
})();
