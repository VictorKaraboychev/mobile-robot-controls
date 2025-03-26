type Point = { x: number, y: number }
type Exit = { position: Point, direction: number }

type Module = { points: Point[], exit: Exit }

const MODULES: { [key: string]: Module } = {
  "straight": {
    points: [
      { x: 0, y: 0 },
      { x: 0, y: 0.1 },
      { x: 0, y: 0.2 },
      { x: 0, y: 0.3 },
    ],
    exit: {
      position: { x: 0, y: 0.3 },
      direction: 0,
    }
  },
  "left": {
    points: [
      { x: 0, y: 0 },
      { x: -0.0018467489107293344, y: 0.023465169756034628 },
      { x: -0.007341522555726981, y: 0.04635254915624211 },
      { x: -0.01634902137174482, y: 0.06809857496093201 },
      { x: -0.028647450843757888, y: 0.08816778784387097 },
      { x: -0.04393398282201787, y: 0.10606601717798211 },
      { x: -0.061832212156129024, y: 0.1213525491562421 },
      { x: -0.08190142503906797, y: 0.13365097862825517 },
      { x: -0.10364745084375787, y: 0.142658477444273 },
      { x: -0.12653483024396536, y: 0.14815325108927066 }
    ],
    exit: {
      position: { x: -0.15, y: 0.15 },
      direction: Math.PI / 2,
    }
  },
  "right": {
    points: [
      { x: 0, y: 0 },
      { x: 0.0018467489107293344, y: 0.023465169756034628 },
      { x: 0.007341522555726981, y: 0.04635254915624211 },
      { x: 0.01634902137174482, y: 0.06809857496093201 },
      { x: 0.028647450843757888, y: 0.08816778784387097 },
      { x: 0.04393398282201787, y: 0.10606601717798211 },
      { x: 0.061832212156129024, y: 0.1213525491562421 },
      { x: 0.08190142503906797, y: 0.13365097862825517 },
      { x: 0.10364745084375787, y: 0.142658477444273 },
      { x: 0.12653483024396536, y: 0.14815325108927066 }
    ],
    exit: {
      position: { x: 0.15, y: 0.15 },
      direction: -Math.PI / 2,
    }
  },
}

const S = MODULES.straight
const L = MODULES.left
const R = MODULES.right


const path: Module[] = [
  S,
  S,
  S,
  S,
  R,
  S,
  S,
  S,
  S,
  R,
  R,
  L,
  L,
  R,
  S,
  S,
  R,
  S,
  R,
  S,
  L,
]

let path_points: Point[] = []

let current_position = { x: 0, y: 0 }
let current_direction = 0

for (const module of path) {
  const last_point = path_points[path_points.length - 1] || current_position
  for (const point of module.points) {
    const x = last_point.x + point.x * Math.cos(current_direction) - point.y * Math.sin(current_direction)
    const y = last_point.y + point.x * Math.sin(current_direction) + point.y * Math.cos(current_direction)
    path_points.push({ x, y })
  }
  current_position = module.exit.position
  current_direction += module.exit.direction
}

console.log(path_points)

const canvas = document.createElement('canvas');
canvas.width = 800;
canvas.height = 800;
document.body.appendChild(canvas);

const ctx = canvas.getContext('2d');
if (ctx) {
  ctx.translate(canvas.width / 2, canvas.height / 2); // Center the canvas
  ctx.scale(100, -100); // Scale for better visibility and invert y-axis

  ctx.beginPath();
  ctx.moveTo(path_points[0].x, path_points[0].y);

  for (const point of path_points) {
    ctx.lineTo(point.x, point.y);
  }

  ctx.strokeStyle = 'blue';
  ctx.lineWidth = 0.01;
  ctx.stroke();
}