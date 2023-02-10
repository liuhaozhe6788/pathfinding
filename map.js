class MyMap {
  constructor() {
    this.IconWidth = 32;
    this.IconHeight = 52;
    this.dotSize = 10;
    // red Icon for source marker
    this.srcIcon = L.icon({
      iconUrl: 'img/red_marker.png',
      iconSize: [this.IconWidth, this.IconHeight], // size of the icon
      iconAnchor: [this.IconWidth / 2, this.IconHeight],
    });

    // green Icon for destination marker
    this.dstIcon = L.icon({
      iconUrl: 'img/green_marker.png',
      iconSize: [this.IconWidth, this.IconHeight], // size of the icon
      iconAnchor: [this.IconWidth / 2, this.IconHeight],
    });

    [this.nodesCoords, this.nodesMap] = this.getNodes();
    this.graph = this.getGraph();
    this.graph.graphPreprocessing(4);
    this.landmarks = [];
    // for(let i = 0; i < this.landmarks.length; i += 2){
    //   console.log(`{${this.landmarks[i].toFixed(7)}, ${this.landmarks[i+1].toFixed(7)}},`);
    // }

    this.map = this.renderMap();
    this.defaultSrcCoords = [30.5908896, 114.2964413];
    this.defaultDstCoords = [30.5804229, 114.2556316];
    this.srcCoords = this.defaultSrcCoords;
    this.dstCoords = this.defaultDstCoords;
    this.srcNode = this.nodesMap[this.srcCoords];
    this.dstNode = this.nodesMap[this.dstCoords];

    this.srcMarker = this.renderSrcMarker();
    this.dstMarker = this.renderDstMarker();

    this.lmMarkers = [];

    this.srcMarker.on('dragend', this.setSrcPos.bind(this));
    this.dstMarker.on('dragend', this.setDstPos.bind(this));

    this.pathLines = new Array();
    this.extPathsLines = new Array();
  }

  getNodes() {
    let nodesCoords = new Array();

    let nodesMap = new Map();

    let node, coords;

    // get nodesCoords array
    for (const nodeFeatures of nodesFeatures) {
      coords = [
        Number(nodeFeatures.properties.lat),
        Number(nodeFeatures.properties.lon),
      ];
      nodesCoords.push(coords);
      node = new Module.Node(coords[0], coords[1]);
      nodesMap[coords] = node;
    }

    return [nodesCoords, nodesMap];
  }

  getGraph() {
    const graph = new Module.Graph();
    let srcNode, dstNode;

    for (let i = 0; i < roadsFeatures.length; i++) {
      srcNode =
        this.nodesMap[
          [Number(roadsFeatures[i].properties.src_la), Number(roadsFeatures[i].properties.src_lo)]
        ];
      dstNode =
        this.nodesMap[
          [Number(roadsFeatures[i].properties.dst_la), Number(roadsFeatures[i].properties.dst_lo)]
        ];
      graph.addEdge(
        srcNode,
        dstNode,
        roadsFeatures[i].properties.length,
        roadsFeatures[i].properties.oneway ? false : true
      );
    }
    return graph;
  }

  getLandmarks(){
    let data = this.graph.getTopnLandmarks(2);
    let landmarks = [];
    for(let i = 0; i < data.size(); i+=2){
      landmarks.push([data.get(i), data.get(i+1)]);
    }
    return landmarks;
  }

  renderMap() {
    // const coords = [latitude, longitude];
    const coords = [30.579557, 114.306074];
    const map = L.map('map').setView(coords, 12);
    map.options.minZoom = 12; // Set minimum zoom

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution:
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    }).addTo(map);

    // Set maximum bounds to Wuhan
    map.setMaxBounds(
      new L.LatLngBounds(
        new L.LatLng(30.408, 113.993),
        new L.LatLng(30.823, 114.648)
      )
    );

    // load geojson data
    L.geoJSON(boundsFeatures, {
      style: function setStyle(feature) {
        return {
          color: 'pink',
          weight: 0,
        };
      },
    }).addTo(map);

    return map;
  }

  renderMarker(coords, icon=null) {
    let marker;
    if(icon){
      marker = L.marker(coords, { icon: icon, draggable: true }).addTo(this.map);
    }
    else{
      marker = L.marker(coords).addTo(this.map);
    }
    return marker;
  }

  renderSrcMarker() {
    return this.renderMarker(this.srcCoords, this.srcIcon);
  }

  renderDstMarker() {
    return this.renderMarker(this.dstCoords, this.dstIcon);
  }

  renderLandmarks(){
    this.landmarks = this.getLandmarks();
    for(let i = 0; i < this.landmarks.length; i ++){
      this.lmMarkers.push(this.renderMarker(this.landmarks[i]));
    }
  }

  setSrcPos() {
    let pos = this.srcMarker.getLatLng();
    let closest = this.nodesCoords.reduce((a, b) =>
      Module.distance(a[0], a[1], pos.lat, pos.lng, 'K') <
      Module.distance(b[0], b[1], pos.lat, pos.lng, 'K') ? a: b
    );
    this.srcMarker.setLatLng(closest);
    this.srcCoords = closest;
    this.srcNode = this.nodesMap[this.srcCoords];
    this.removePaths();
  }

  setDstPos() {
    let pos = this.dstMarker.getLatLng();
    let closest = this.nodesCoords.reduce((a, b) =>
      Module.distance(a[0], a[1], pos.lat, pos.lng, 'K') <
      Module.distance(b[0], b[1], pos.lat, pos.lng, 'K') ? a: b
    );
    this.dstMarker.setLatLng(closest);
    this.dstCoords = closest;
    this.dstNode = this.nodesMap[this.dstCoords];
    this.removePaths();
  }

  renderLine(srcCoords, dstCoords, color) {
    let lineString = L.polyline([srcCoords, dstCoords]).addTo(this.map);
    lineString.setStyle({ color: color });
    return lineString;
  }

  renderAllPaths(algo){
    this.removePaths();
    let success;
    this.removeLmMarkers();
    const start = Date.now();
    switch(algo){
      case "Uniform Cost Search":
        success = this.graph.UCSearch(this.srcNode, this.dstNode);
        break;
      case "A* Search":
        success = this.graph.AStarSearch(this.srcNode, this.dstNode);
        break;
      case "ALT Search":
        success = this.graph.ALTSearch(this.srcNode, this.dstNode);
        break;
      default:
        break;
    }
    const end = Date.now();
    window.confirm(`Execution time: ${(end - start)/1000} s`);
    if(algo==="ALT Search"){
      this.renderLandmarks();
    }
    this.renderExtPaths("blue");
    if(success)
      this.renderFinalPath("red");
  }

  renderExtPaths(color){
    let extPaths = this.graph.getExtPaths();
    let srcCoords, dstCoords;
    let extPathCoords = [];
    for (let path = 0; path < extPaths.size(); path+=4){
        srcCoords = [extPaths.get(path), extPaths.get(path+1)];
        dstCoords = [extPaths.get(path+2), extPaths.get(path+3)];
        extPathCoords.push([srcCoords, dstCoords]);
    }
    extPaths.delete();
    for(const pathCoords of extPathCoords)
      this.extPathsLines.push(L.polyline(pathCoords, { color: color}).addTo(this.map));
  }

  renderFinalPath(color) {
    this.finalPath = this.graph.getPath(this.srcNode, this.dstNode);
    let srcCoords, dstCoords;
    for (let i = 0; i <= this.finalPath.size() - 4; i+=2) {
        srcCoords = [this.finalPath.get(i), this.finalPath.get(i+1)];
        dstCoords = [this.finalPath.get(i+2), this.finalPath.get(i+3)];
        this.pathLines.push(L.polyline([srcCoords, dstCoords], { color: color}).addTo(this.map));
    }
    this.finalPath.delete();

  }

  removePaths() {
    this.extPathsLines.forEach((val) => {val.remove(); val = null;});
    this.extPathsLines = [];
    this.pathLines.forEach((val) => {val.remove(); val = null;});
    this.pathLines = [];
  }

  removeLmMarkers(){
    this.lmMarkers.forEach((val) => {val.remove(); val = null;});
    this.lmMarkers = [];
  }

  resetMap(){
    // set start and end marker and node
    this.srcMarker.setLatLng(this.defaultSrcCoords);
    this.dstMarker.setLatLng(this.defaultDstCoords);
    this.srcNode = this.nodesMap[this.defaultSrcCoords];
    this.dstNode = this.nodesMap[this.defaultDstCoords];

    // set map view
    const coords = [30.579557, 114.306074];
    this.map.setView(coords, 12);
  }

  getUniqueEl(data){
    data = new Set(data.map(JSON.stringify));
    data = Array.from(data).map(JSON.parse);
    return data;
  }
}

function testEmbind() {
  console.log('Start testing embind code');
  let nodes = new Array();
  let graph = new Module.Graph();
  const num = 180000;
  for (let i = 0; i < num; i++) {
    nodes[i] = new Module.Node(i, i * i);
  }
  for (let i = 0; i < num - 1; i++) {
    graph.addEdge(nodes[i], nodes[i + 1], 1, true);
  }
  for(let n = 0; n < 200; n++){
    if (graph.UCSearch(nodes[0], nodes[num - 1])) {
      console.log(n);
      let extPaths = graph.getExtPaths();
      let path = graph.getPath(nodes[0], nodes[num - 1]);
      // for (let i = 0; i < path.size()-1; i+=2){
      //     console.log(path.get(i), path.get(i+1));
      // }
      extPaths.delete();
      path.delete();
    }

    if (graph.AStarSearch(nodes[0], nodes[num - 1])) {
      console.log(n);
      let extPaths = graph.getExtPaths();
      let path = graph.getPath(nodes[0], nodes[num - 1]);
      // for (let i = 0; i < path.size()-1; i+=2){
      //     console.log(path.get(i), path.get(i+1));
      // }
      extPaths.delete();
      path.delete();
    }
  }
 
  console.log('Testing embind code completed');
}
