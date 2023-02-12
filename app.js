const fullTrigBtnEl = document.querySelector('.full-trig-btn');
const rstTrigBtnEl = document.querySelector('.rst-trig-btn');
const selectEl = document.querySelector(".sel");

class App {
  constructor() {
    this.myMap = new MyMap();
    this.algorithm = "Uniform Cost Search";
    selectEl.addEventListener('change', this.selAlgo.bind(this));
    fullTrigBtnEl.addEventListener('click', this.start.bind(this));
    rstTrigBtnEl.addEventListener('click', this.reset.bind(this));
  }

  start() {
    this.myMap.renderAllPaths(this.algorithm);
  }

  selAlgo(){
    let index = selectEl.selectedIndex;
    this.algorithm = selectEl.options[index].text;
  }

  reset(){
    this.myMap.removeLmMarkers();
    this.myMap.removePaths();
    this.myMap.resetMap();
  }
}
