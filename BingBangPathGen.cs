using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace GUI_Interface.TrajectoryAndObstacleAvoidance
{
	class BingBangPathGen
	{
		public const float d = 5f; // offset used to move along the bing bang wall

		public static RobotState getNewDest(Map map, RobotState state, RobotState dest){
			float pathSlope; // slope of current path
			float bingSlope; // bing bang slope where "bing bang" is alway perpendicular to the current path
			Line bingbang = null; // bing bang wall which is represented by a Line object
			if (state.X == dest.X)
			{
				// current path is vertical
				bingSlope = 0;
				bingbang = new Line(bingSlope, dest.Y, false);
			} else if (state.Y == dest.Y) 
			{
				// current path is horizontal
				pathSlope = 0;
				bingbang = new Line(0, dest.X, true);
			} else {
				// normal case
				pathSlope = (dest.Y - state.Y) / (dest.X - state.X);
				bingSlope = -1 / pathSlope; // k1*k2 = -1  <=> two lines are perpendicular
				float b = dest.Y - bingSlope * dest.X;
				bingbang = new Line(bingSlope, b, false);	
			}
            // TODO fix Aaron hack - should be RobotState or something general, not R2D2 state.
            RobotState currDest = new R2D2State { X = dest.X, Y = dest.Y, T = 0f };
            RobotState currDest1 = new R2D2State { X = dest.X, Y = dest.Y, T = 0f }; // temporary destination state with positive offset (offset added later)
            RobotState currDest2 = new R2D2State { X = dest.X, Y = dest.Y, T = 0f }; // temporary destination state with negative offset (offset added later)
			bool hasIntersect;
			bool usingPositiveOffset = true;
			float offsetX, offsetY;
			while (hasIntersect = checkCollision(map, state, currDest)) {
				if (usingPositiveOffset)
				{
					if (bingbang.isVertical)
					{
						offsetX = 0;
						offsetY = d;
					} 
					else
					{
						offsetX = (float) (d / Math.Sqrt(1 + bingbang.k * bingbang.k));
                        offsetY = (float) (d * bingbang.k / Math.Sqrt(1 + bingbang.k * bingbang.k));
					}
					currDest1.X += offsetX;
					currDest1.Y += offsetY;
					currDest = currDest1;
					usingPositiveOffset = false;
				} 
				else
				{
					if (bingbang.isVertical)
					{
						offsetX = 0;
						offsetY = -d;
					} 
					else
					{
						offsetX = (float) (-d / Math.Sqrt(1 + bingbang.k * bingbang.k));
						offsetY = (float) (-d * bingbang.k / Math.Sqrt(1 + bingbang.k * bingbang.k));
					}
					currDest2.X += offsetX;
					currDest2.Y += offsetY;
					currDest = currDest2;
					usingPositiveOffset = true;
				}
			}
			
			return currDest;
		}
		
		public static bool checkCollision(Map map, RobotState state, RobotState dest) {
            float radius = ((R2D2State)state).ScreenRadius;
			foreach (Obstacle obs in map) {
                Obstacle ob = obs.createExpandedObstacle(Constants.BING_BANG_OBSTACLE_PROXIMITY_MULT * radius);
                if (CollisionDetection.lineSegmentPolyIntersect(state.X, state.Y, dest.X, dest.Y, (BShape_Poly)ob))
                {
                    return true;
                }
			}
			return false;
		}
	}
}