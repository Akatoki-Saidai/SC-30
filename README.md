# SC-30
Noshiro_Space_Event_22th <br>
ricefield君を崇めましょう
rice<br>
<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>テトリス</title>

<style>
    body{
        margin:0;
        background:#111;
        color:white;
        display:flex;
        justify-content:center;
        align-items:center;
        height:100vh;
        font-family:sans-serif;
    }

    .container{
        display:flex;
        gap:20px;
        align-items:flex-start;
    }

    canvas{
        background:#000;
        border:3px solid #fff;
    }

    .info{
        font-size:20px;
    }

    button{
        margin-top:10px;
        padding:10px 20px;
        font-size:16px;
        cursor:pointer;
    }
</style>
</head>
<body>

<div class="container">
    <canvas id="game" width="300" height="600"></canvas>

    <div class="info">
        <div>Score: <span id="score">0</span></div>

        <div style="margin-top:20px;">
            操作方法
            <ul>
                <li>← → : 移動</li>
                <li>↓ : 高速落下</li>
                <li>↑ : 回転</li>
            </ul>
        </div>

        <button onclick="resetGame()">リスタート</button>
    </div>
</div>

<script>
const canvas = document.getElementById("game");
const ctx = canvas.getContext("2d");

const ROW = 20;
const COL = 10;
const BLOCK = 30;

ctx.scale(BLOCK, BLOCK);

const scoreEl = document.getElementById("score");

const colors = [
    null,
    "cyan",
    "blue",
    "orange",
    "yellow",
    "green",
    "purple",
    "red"
];

const pieces = [
    [],
    [[1,1,1,1]],

    [[2,0,0],
     [2,2,2]],

    [[0,0,3],
     [3,3,3]],

    [[4,4],
     [4,4]],

    [[0,5,5],
     [5,5,0]],

    [[0,6,0],
     [6,6,6]],

    [[7,7,0],
     [0,7,7]]
];

function createBoard(w,h){
    return Array.from({length:h},()=>Array(w).fill(0));
}

const board = createBoard(COL, ROW);

const player = {
    pos:{x:0,y:0},
    matrix:null,
    score:0
};

function drawMatrix(matrix, offset){
    matrix.forEach((row,y)=>{
        row.forEach((value,x)=>{
            if(value !== 0){
                ctx.fillStyle = colors[value];
                ctx.fillRect(
                    x + offset.x,
                    y + offset.y,
                    1,
                    1
                );

                ctx.strokeStyle = "#111";
                ctx.strokeRect(
                    x + offset.x,
                    y + offset.y,
                    1,
                    1
                );
            }
        });
    });
}

function draw(){
    ctx.fillStyle = "#000";
    ctx.fillRect(0,0,canvas.width,canvas.height);

    drawMatrix(board,{x:0,y:0});
    drawMatrix(player.matrix,player.pos);
}

function merge(board, player){
    player.matrix.forEach((row,y)=>{
        row.forEach((value,x)=>{
            if(value !== 0){
                board[y + player.pos.y][x + player.pos.x] = value;
            }
        });
    });
}

function collide(board, player){
    const m = player.matrix;
    const o = player.pos;

    for(let y=0; y<m.length; y++){
        for(let x=0; x<m[y].length; x++){
            if(
                m[y][x] !== 0 &&
                (
                    board[y + o.y] &&
                    board[y + o.y][x + o.x]
                ) !== 0
            ){
                return true;
            }
        }
    }
    return false;
}

function rotate(matrix){
    return matrix[0].map((_,i)=>
        matrix.map(row=>row[i]).reverse()
    );
}

function playerRotate(){
    const pos = player.pos.x;
    let offset = 1;

    player.matrix = rotate(player.matrix);

    while(collide(board, player)){
        player.pos.x += offset;
        offset = -(offset + (offset > 0 ? 1 : -1));

        if(offset > player.matrix[0].length){
            player.matrix = rotate(
                rotate(
                    rotate(player.matrix)
                )
            );
            player.pos.x = pos;
            return;
        }
    }
}

function playerDrop(){
    player.pos.y++;

    if(collide(board, player)){
        player.pos.y--;
        merge(board, player);
        clearLines();
        playerReset();

        if(collide(board, player)){
            alert("ゲームオーバー");
            resetGame();
        }
    }

    dropCounter = 0;
}

function playerMove(dir){
    player.pos.x += dir;

    if(collide(board, player)){
        player.pos.x -= dir;
    }
}

function clearLines(){
    let rowCount = 1;

    outer:
    for(let y=board.length -1; y>=0; y--){
        for(let x=0; x<board[y].length; x++){
            if(board[y][x] === 0){
                continue outer;
            }
        }

        const row = board.splice(y,1)[0].fill(0);
        board.unshift(row);
        y++;

        player.score += rowCount * 100;
        rowCount *= 2;
    }

    scoreEl.textContent = player.score;
}

function playerReset(){
    const rand =
        Math.floor(Math.random() * 7) + 1;

    player.matrix = pieces[rand];

    player.pos.y = 0;
    player.pos.x =
        ((COL / 2) | 0) -
        ((player.matrix[0].length / 2) | 0);
}

let dropCounter = 0;
let dropInterval = 500;
let lastTime = 0;

function update(time = 0){
    const delta = time - lastTime;
    lastTime = time;

    dropCounter += delta;

    if(dropCounter > dropInterval){
        playerDrop();
    }

    draw();

    requestAnimationFrame(update);
}

document.addEventListener("keydown", event=>{
    if(event.key === "ArrowLeft"){
        playerMove(-1);
    }
    else if(event.key === "ArrowRight"){
        playerMove(1);
    }
    else if(event.key === "ArrowDown"){
        playerDrop();
    }
    else if(event.key === "ArrowUp"){
        playerRotate();
    }
});

function resetGame(){
    board.forEach(row=>row.fill(0));
    player.score = 0;
    scoreEl.textContent = 0;
    playerReset();
}

resetGame();
update();
</script>

</body>
</html>