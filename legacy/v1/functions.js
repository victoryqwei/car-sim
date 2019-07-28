function drawCircle(x, y, r, o) {
	ctx.beginPath();
	ctx.arc(x, y, r, 0, 2 * Math.PI, false);
    ctx.fillStyle = 'grey';
    ctx.globalAlpha = o || 0.2;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.lineWidth = 0.5;
    ctx.strokeStyle = 'black';
    ctx.stroke();
    ctx.closePath();
}

function drawRect(x, y, w, h, d, o) {
	ctx.translate(x, y)
	ctx.rotate(d);

	ctx.rect(-w/2, -h/2, w, h);
	ctx.fillStyle = o || 'grey';
	ctx.globalAlpha = 1;
	ctx.fill();
	ctx.beginPath();
	ctx.lineWidth = 0.5;
	ctx.strokeStyle = 'black';
	ctx.stroke();

	ctx.resetTransform();

}

function drawImage(img, x, y, w, h, angle) {
	ctx.translate(x, y);
	ctx.rotate(angle);
	ctx.drawImage(img, -w, -h, w*2, h*2);
	ctx.resetTransform();
}

function drawCircleImage(img, x, y, r, angle) {
	ctx.translate(x, y)
	ctx.rotate(angle);
	ctx.drawImage(img, -r, -r, r*2, r*2);
	ctx.resetTransform();
}

function drawLine(x1, y1, x2, y2) {
	ctx.beginPath();
	ctx.lineWidth = 2;
	ctx.moveTo(x1, y1);
	ctx.lineTo(x2, y2);
	ctx.stroke();
	ctx.closePath();
}

function constrain(value, a, b) {
	if (value < a) {
		return a;
	} else if (value > b) {
		return b;
	} else {
		return value
	}
}

function random(max, min) {
	return Math.random() * (max - min) + min;
}

function dist(a, b) {
	return Math.sqrt(Math.pow(a.x-b.x, 2) + Math.pow(a.y-b.y, 2));
}