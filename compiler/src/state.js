class State {
  constructor(state) {
    for (let k in state) {
      this[k] = state[k];
    }
  }

  get noteLengthFrames() {
    return Math.round(this._noteLengthExactFrames());
  }

  get quantLengthFrames() {
    // return Math.round(this._noteLengthExactFrames() * (this.quantLength / 8));
    return Math.round(this._noteLengthExactFrames());
  }

  get borrowFrames() {
    let exact = this._noteLengthExactFrames();
    return exact - Math.round(exact);
  }

  _noteLengthExactFrames() {
    // returns the number of frames in a whole minute, 
    // taking into account tempo and the length of each note
    // 10811 = 60s / 5.55ms (111 ticks of timer0, at 20KHz, 50us each tick)
    // 3604 = 60s / 5.55ms (333 ticks of timer0, at 20KHz, 50us each tick)
    return 3604 / this.tempo / this.noteLength * 4;
  }
}

module.exports = State;
