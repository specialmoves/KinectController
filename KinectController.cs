using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Drawing;
using AForge.Imaging;
using AForge.Imaging.Filters;
using AForge.Math.Geometry;
using AForge;

namespace MapPanner
{
	/**
	 * Used to handle the RGB, depth and skeletal streams from the kinect.
	 * Can also perform depth filtering and point tracking on depth data.
	 */
	class KinectController
	{
		// Constants
		private const int DEFAULT_DEPTH_RANGE = 1860; // How far is the depth we wish to filter?
		private const int DEFAULT_DEPTH_PADDING = 30; // How much of that range do we want to filter?

		private const int MIN_POINT_WIDTH = 4; 		// Minimum point width if using point tracking
		private const int MAX_POINT_WIDTH = 100;	// ... and the max point width.
		
		private const ObjectsOrder OBJECTS_ORDER = ObjectsOrder.XY;

		// Kinect
		public KinectSensor KinectSensor;

		// Events
		public EventHandler ColourFrameReady;
		public EventHandler DepthFrameReady;
		public EventHandler SkeletonFrameReady;

		// Data
		public byte[] ColourData;		// The RGB camera data
		public byte[] DepthRangeData;	// The filtered depth data
		public short[] DepthData;		// The raw depth data
		public Skeleton[] SkeletonData; // The skeleton data

		// Public Properties
		public int DepthRange;
		public int DepthPadding;
		public List<System.Drawing.Point> Points;
		public Rectangle trackingRegion;
		public Bitmap regionedBmp;

		// Private Properties
		private BlobCounter pointTracker;
		private bool filterRange;
		private bool trackPoints;
		private int depthWidth;
		private int depthHeight;
		private Pixellate filter;

		public KinectController(KinectSensor sensor)
		{
			KinectSensor = sensor;

			filter = new Pixellate(2);
			filterRange = false;
			DepthRange = DEFAULT_DEPTH_RANGE;
			DepthPadding = DEFAULT_DEPTH_PADDING;
		}

		public void Start()
		{
			KinectSensor.Start();
		}
		
		public void Stop()
		{
			KinectSensor.Stop();
		}

		public void EnableColourStream(ColorImageFormat format)
		{
			KinectSensor.ColorStream.Enable(format);
			KinectSensor.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinectSensor_ColorFrameReady);
		}

		public void EnableSkeletonStream()
		{
			KinectSensor.SkeletonStream.Enable();
			KinectSensor.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(KinectSensor_SkeletonFrameReady);
		}

		public void EnableDepthStream(DepthImageFormat format, int width, int height, bool filterRange = false, bool trackPoints = false)
		{
			this.filterRange = filterRange;
			this.trackPoints = trackPoints;
			this.depthWidth = width;
			this.depthHeight = height;

			if (trackPoints)
			{
				pointTracker = new BlobCounter();
				pointTracker.MinWidth = MIN_POINT_WIDTH;
				pointTracker.MinHeight = MIN_POINT_WIDTH;
				pointTracker.MaxWidth = MAX_POINT_WIDTH;
				pointTracker.MaxHeight = MAX_POINT_WIDTH;
				pointTracker.ObjectsOrder = OBJECTS_ORDER;
				pointTracker.FilterBlobs = true;

				Points = new List<System.Drawing.Point>();
			}

			KinectSensor.DepthStream.Enable(format);
			KinectSensor.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(kinectSensor_DepthFrameReady);
		}
		
		private void kinectSensor_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
		{
			using (ColorImageFrame imageFrame = e.OpenColorImageFrame())
			{
				if (imageFrame == null) return;

				if (ColourData == null) ColourData = new byte[imageFrame.PixelDataLength];
				imageFrame.CopyPixelDataTo(ColourData);
				if (ColourFrameReady != null) ColourFrameReady(this, EventArgs.Empty);
			}
		}

		private void KinectSensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
		{
			bool recievedData = false;

			using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
			{
				SkeletonData = new Skeleton[KinectSensor.SkeletonStream.FrameSkeletonArrayLength];
				if (skeletonFrame != null)
				{
					recievedData = true;
					skeletonFrame.CopySkeletonDataTo(SkeletonData);
				}
			}

			if (recievedData && SkeletonFrameReady != null)
			{
				SkeletonFrameReady(this, EventArgs.Empty);
			}
		}

		private void kinectSensor_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
		{
			using (DepthImageFrame imageFrame = e.OpenDepthImageFrame())
			{
				if (imageFrame == null) return;

				// Save the depth data from the kinect
				if (DepthData == null) DepthData = new short[imageFrame.PixelDataLength];
				imageFrame.CopyPixelDataTo(DepthData);
			}

			if (filterRange)
			{
				FilterDepthRange();
			}

			// Dispatch a notificaiton event
			if (DepthFrameReady != null) DepthFrameReady(this, EventArgs.Empty);
		}
		
		/**
		 * Filters the depth data to show our specified range
		 */
		private void FilterDepthRange()
		{
			if (DepthRangeData == null) DepthRangeData = new byte[DepthData.Length << 2];

			uint layerPointColor;
			byte rChan, gChan, bChan, aChan;
			int rgbCount = 0;
			int i = 0;
			int depthValue; // This will hold the individual pixel distance

			while (i < DepthData.Length)
			{
				depthValue = DepthData[i];	// The raw value
				depthValue >>= 3;			// Chop off the three bits which hold the player id

				if (depthValue > DepthRange - DepthPadding && depthValue < DepthRange)
				{
					layerPointColor = 0xDD000000; // Closer than target range
				}
				else if (depthValue >= DepthRange && depthValue < DepthRange + DepthPadding)
				{
					layerPointColor = 0xFF00FF00; // Target range
				}
				else if (depthValue >= DepthRange + DepthPadding && depthValue < DepthRange + (DepthPadding << 1))
				{
					layerPointColor = 0xCC000000; // Further than target range
				}
				else
				{
					layerPointColor = 0xFF000000; // Nowhere near target range
				}

				// Get our colour channels
				aChan = (byte)((layerPointColor >> 24) & 0xFF);
				rChan = (byte)((layerPointColor >> 16) & 0xFF);
				gChan = (byte)((layerPointColor >> 8) & 0xFF);
				bChan = (byte)(layerPointColor & 0xFF);

				// Used by our native bitmap and point tracking display
				DepthRangeData[rgbCount] = rChan;
				DepthRangeData[rgbCount + 1] = gChan;
				DepthRangeData[rgbCount + 2] = bChan;
				DepthRangeData[rgbCount + 3] = aChan;

				i++;
				rgbCount += 4;
			}

			if (trackPoints) TrackPoints();
		}
		
		private void TrackPoints()
		{
			int w, h, x, y;

			if (trackingRegion.Width == 0)
			{
				w = depthWidth;
				h = depthHeight;
				x = 0;
				y = 0;
			}
			else
			{
				w = trackingRegion.Width;
				h = trackingRegion.Height;
				x = trackingRegion.X;
				y = trackingRegion.Y;
			}

			Bitmap bmp = new Bitmap(depthWidth, depthHeight, PixelFormat.Format32bppPArgb);

			//Create a BitmapData and Lock all pixels to be written 
			BitmapData bmpData = bmp.LockBits(new Rectangle(0, 0, bmp.Width, bmp.Height), ImageLockMode.WriteOnly, bmp.PixelFormat);

			//Copy the data from the byte array into BitmapData.Scan0
			Marshal.Copy(DepthRangeData, 0, bmpData.Scan0, DepthRangeData.Length);

			//Unlock the pixels
			bmp.UnlockBits(bmpData);

			if(regionedBmp == null)
				regionedBmp = new Bitmap(w, h, PixelFormat.Format24bppRgb);
 
			Graphics g = Graphics.FromImage(regionedBmp);
			g.DrawImage(bmp, new Rectangle(0, 0, w, h), new Rectangle(x, y, w, h), GraphicsUnit.Pixel);
			g.Dispose();

			filter.ApplyInPlace(regionedBmp);

			// Get our tracked blobs and then save them
			pointTracker.ProcessImage(regionedBmp);
			ProcessTrackedPoints();
		}

		/**
		 * Loops through our depth range filtered data and saves our tracked points 
		 */
		private void ProcessTrackedPoints()
		{
			System.Drawing.Rectangle[] rects = pointTracker.GetObjectsRectangles();

			Points.Clear();

			float xPos, yPos;

			for (int i = 0; i < rects.Length; i++)
			{
				xPos = (((float)rects[i].X + ((float)rects[i].Width / 2)) / (float)trackingRegion.Width) * depthWidth;
				yPos = (((float)rects[i].Y + ((float)rects[i].Height / 2)) / (float)trackingRegion.Height) * depthHeight;

				Points.Add(new System.Drawing.Point((int)xPos, (int)yPos));
			}

			// create convex hull searching algorithm
			GrahamConvexHull hullFinder = new GrahamConvexHull();

			// lock image to draw on it
			BitmapData data = regionedBmp.LockBits(
				new Rectangle(0, 0, regionedBmp.Width, regionedBmp.Height),
					ImageLockMode.ReadWrite, regionedBmp.PixelFormat);

			// process each blob
			foreach (Blob blob in pointTracker.GetObjectsInformation())
			{
				List<IntPoint> edgePoints = new List<IntPoint>();

				List<IntPoint> leftPoints, rightPoints;

				// get blob's edge points
				pointTracker.GetBlobsLeftAndRightEdges(blob,
					out leftPoints, out rightPoints);

				edgePoints.AddRange(leftPoints);
				edgePoints.AddRange(rightPoints);

				// blob's convex hull
				List<IntPoint> hull = hullFinder.FindHull(edgePoints);

				Drawing.Polygon(data, hull, Color.Red);
			}

			regionedBmp.UnlockBits(data);
		}
	}
}