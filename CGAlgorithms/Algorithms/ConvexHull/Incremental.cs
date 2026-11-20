using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class Incremental : Algorithm
    {
        private struct HullPoint : IComparable<HullPoint>
        {
            public Point P;
            public double Angle;
            public int Edge;

            public HullPoint(Point p, double angle, int tieBreaker = 0)
            {
                P = p;
                Angle = angle;
                Edge = tieBreaker;
            }

            public int CompareTo(HullPoint other)
            {
                int cmp = Angle.CompareTo(other.Angle);
                return cmp != 0 ? cmp : Edge.CompareTo(other.Edge);
            }
        }

        private HullPoint Prev(SortedSet<HullPoint> hull, double angle)
        {
            var key = new HullPoint(null, angle, -1);
            if (key.CompareTo(hull.Min) < 0) return hull.Max;
            return hull.GetViewBetween(hull.Min, key).Max;
        }

        private HullPoint Next(SortedSet<HullPoint> hull, double angle)
        {
            var key = new HullPoint(null, angle, 1);
            if (key.CompareTo(hull.Max) > 0) return hull.Min;
            return hull.GetViewBetween(key, hull.Max).Min;
        }

        private double Distance(Point p, double cx, double cy)
        {
            double dx = p.X - cx;
            double dy = p.Y - cy;
            return dx * dx + dy * dy;
        }

        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            var uniquePointsTmp = new HashSet<Tuple<double, double>>();
            var pts = new List<Point>();
            foreach (Point p in points)
            {
                var cur = new Tuple<double, double>(p.X, p.Y);
                if (uniquePointsTmp.Contains(cur)) continue;
                uniquePointsTmp.Add(cur);
                pts.Add(p);
            }

            if (pts.Count <= 2)
            {
                outPoints = new List<Point>(pts);
                return;
            }

            pts.Sort((a, b) =>
            {
                int cmpX = a.X.CompareTo(b.X);
                return cmpX != 0 ? cmpX : a.Y.CompareTo(b.Y);
            });

            Point p1 = pts[0];
            Point p2 = pts[1];
            Point p3 = null;
            for (int i = 2; i < pts.Count; i++)
            {
                if (HelperMethods.CheckTurn(new Line(p1, p2), pts[i]) != Enums.TurnType.Colinear)
                {
                    p3 = pts[i];
                    break;
                }
            }

            if (p3 == null)
            {
                outPoints = new List<Point> { pts.First(), pts.Last() };
                return;
            }

            double cx = (p1.X + p2.X + p3.X) / 3.0;
            double cy = (p1.Y + p2.Y + p3.Y) / 3.0;

            var hull = new SortedSet<HullPoint>{
                new HullPoint(p1, Math.Atan2(p1.Y - cy, p1.X - cx)),
                new HullPoint(p2, Math.Atan2(p2.Y - cy, p2.X - cx)),
                new HullPoint(p3, Math.Atan2(p3.Y - cy, p3.X - cx)),
            };

            for (int i = 2; i < pts.Count; i++)
            {
                Point p = pts[i];
                double angle = Math.Atan2(p.Y - cy, p.X - cx);
                var cand = new HullPoint(p, angle);

                if (p.Equals(p3)) continue;

                if (hull.Contains(cand))
                {
                    var found = hull.GetViewBetween(cand, cand).Min;
                    if (Distance(p, cx, cy) <= Distance(found.P, cx, cy))
                        continue;
                    hull.Remove(found);
                }
                else
                {
                    HullPoint prevCheck = Prev(hull, angle);
                    HullPoint nextCheck = Next(hull, angle);

                    if (HelperMethods.CheckTurn(new Line(prevCheck.P, nextCheck.P), p) != Enums.TurnType.Right)
                        continue;
                }

                HullPoint next = Next(hull, angle);
                while (hull.Count >= 3)
                {
                    HullPoint next2 = Next(hull, next.Angle);
                    if (HelperMethods.CheckTurn(new Line(p, next.P), next2.P) != Enums.TurnType.Left)
                    {
                        hull.Remove(next);
                        next = next2;
                    }
                    else break;
                }

                HullPoint prev = Prev(hull, angle);
                while (hull.Count >= 3)
                {
                    HullPoint prev2 = Prev(hull, prev.Angle);
                    if (HelperMethods.CheckTurn(new Line(prev2.P, prev.P), p) != Enums.TurnType.Left)
                    {
                        hull.Remove(prev);
                        prev = prev2;
                    }
                    else break;
                }

                hull.Add(cand);
            }

            outPoints = new List<Point>(hull.Count);
            foreach (var h in hull)
            {
                outPoints.Add(h.P);
            }
        }

        public override string ToString()
        {
            return "Convex Hull - Incremental";
        }
    }
}
