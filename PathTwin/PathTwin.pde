// Processing 4 / Java Mode. Uses the course's original motor and Euler models.
import java.util.Locale;

final int DT = 10, SAMPLE_MS = 50;
final float WHEELBASE_MM = 90;
Table plan;
MotorsModel_c motors;
OdometryModel_c odometry;
ArrayList<PVector> trail;
ArrayList<String> log;
int elapsed, total, lastFrame;
boolean running;
String phase = "ready";

void settings() { size(1000, 720); }

void setup() {
  plan = loadTable("path.csv", "header");
  for (TableRow row : plan.rows()) total += row.getInt("duration_ms");
  resetTrial();
  surface.setTitle("Path Reproduction | Processing Digital Twin");
  // Reproducible batch mode still executes the Processing sketch and course models.
  if ("1".equals(System.getenv("PATH_TWIN_BATCH"))) {
    while (running) stepTrial();
    exit();
  }
}

void resetTrial() {
  motors = new MotorsModel_c();
  odometry = new OdometryModel_c(16, 358.3);
  trail = new ArrayList<PVector>();
  trail.add(new PVector(0, 0));
  log = new ArrayList<String>();
  log.add("t_ms,segment,left_pwm,right_pwm,left_count,right_count,x_mm,y_mm,theta_rad");
  elapsed = 0;
  lastFrame = millis();
  running = true;
  appendSample(segmentAt(0));
}

TableRow segmentAt(int t) {
  for (TableRow row : plan.rows()) {
    int duration = row.getInt("duration_ms");
    if (t < duration) return row;
    t -= duration;
  }
  return null;
}

void appendSample(TableRow row) {
  RobotPose_c p = odometry.pose;
  String name = row == null ? "done" : row.getString("name");
  int left = row == null ? 0 : row.getInt("left_pwm");
  int right = row == null ? 0 : row.getInt("right_pwm");
  log.add(String.format(Locale.ROOT, "%d,%s,%d,%d,%.6f,%.6f,%.4f,%.4f,%.7f",
    elapsed, name, left, right, odometry.encoders.left, odometry.encoders.right,
    p.x, p.y, p.theta));
}

void stepTrial() {
  TableRow row = segmentAt(elapsed);
  phase = row.getString("name");
  motors.setPWM(row.getInt("left_pwm"), row.getInt("right_pwm"));
  motors.update();
  odometry.update(DT * motors.left.speedMmPerMs, DT * motors.right.speedMmPerMs, WHEELBASE_MM);
  elapsed += DT;
  RobotPose_c p = odometry.pose;
  trail.add(new PVector(p.x, p.y));
  if (elapsed % SAMPLE_MS == 0) appendSample(segmentAt(elapsed));
  if (elapsed == total) {
    running = false;
    phase = "done";
    motors.setPWM(0, 0);
    saveStrings(sketchPath("sim_run.csv"), log.toArray(new String[0]));
    println("Saved " + sketchPath("sim_run.csv"));
  }
}

void draw() {
  int now = millis();
  while (running && now - lastFrame >= DT) {
    stepTrial();
    lastFrame += DT;
  }
  background(247, 249, 252);
  fill(25, 37, 55);
  textSize(26);
  text("Path Reproduction", 28, 40);
  textSize(15);
  text("Processing 4 | course motor + Euler models | R: replay", 28, 65);
  text(String.format(Locale.ROOT, "%.2f / %.2f s   %s", elapsed/1000.0, total/1000.0, phase), 28, 90);
  pushMatrix();
  translate(160, 580);
  scale(1.15, -1.15);
  stroke(170);
  line(-60, 0, 670, 0);
  line(0, -65, 0, 300);
  stroke(41, 105, 190);
  strokeWeight(2);
  noFill();
  beginShape();
  for (PVector p : trail) vertex(p.x, p.y);
  endShape();
  RobotPose_c p = odometry.pose;
  translate(p.x, p.y);
  rotate(p.theta);
  fill(242, 159, 64);
  ellipse(0, 0, 35, 35);
  line(0, 0, 25, 0);
  popMatrix();
  fill(25, 37, 55);
  text("+x", 935, 585);
  text("+y", 150, 120);
  text(String.format(Locale.ROOT, "x %.1f mm   y %.1f mm   heading %.1f°", p.x, p.y, degrees(p.theta)), 28, 690);
}

void keyPressed() {
  if (key == 'r' || key == 'R') resetTrial();
}
