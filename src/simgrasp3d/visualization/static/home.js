(() => {
  const readout = document.getElementById('schematic-readout');
  const title = readout.querySelector('strong');
  const copy = readout.querySelector('span:last-child');
  document.querySelectorAll('.system-node').forEach((node) => {
    const show = () => {
      title.textContent = node.dataset.title;
      copy.textContent = node.dataset.copy;
    };
    node.addEventListener('mouseenter', show);
    node.addEventListener('focus', show);
  });
})();
