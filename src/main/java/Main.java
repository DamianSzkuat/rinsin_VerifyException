

import static com.google.common.collect.Lists.newArrayList;
import static com.google.common.collect.Maps.newHashMap;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Map;

import javax.annotation.Nullable;
import javax.measure.unit.SI;

import org.apache.commons.math3.random.RandomGenerator;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Monitor;

import com.github.rinde.rinsim.core.Simulator;
import com.github.rinde.rinsim.core.model.comm.CommModel;
import com.github.rinde.rinsim.core.model.pdp.DefaultPDPModel;
import com.github.rinde.rinsim.core.model.pdp.Depot;
import com.github.rinde.rinsim.core.model.pdp.PDPModel;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.ParcelDTO;
import com.github.rinde.rinsim.core.model.pdp.PDPModel.ParcelState;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.road.RoadModelBuilders;
import com.github.rinde.rinsim.core.model.time.TickListener;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.event.Listener;
import com.github.rinde.rinsim.geom.Graph;
import com.github.rinde.rinsim.geom.Graphs;
import com.github.rinde.rinsim.geom.LengthData;
import com.github.rinde.rinsim.geom.ListenableGraph;
import com.github.rinde.rinsim.geom.MultiAttributeData;
import com.github.rinde.rinsim.geom.Point;
import com.github.rinde.rinsim.geom.TableGraph;
import com.github.rinde.rinsim.geom.io.DotGraphIO;
import com.github.rinde.rinsim.geom.io.Filters;
import com.github.rinde.rinsim.ui.View;
import com.github.rinde.rinsim.ui.renderers.AGVRenderer;
import com.github.rinde.rinsim.ui.renderers.CommRenderer;
import com.github.rinde.rinsim.ui.renderers.WarehouseRenderer;
import com.google.common.collect.ImmutableTable;
import com.google.common.collect.Lists;
import com.google.common.collect.Table;

import project.agents.AgvAgent;
import project.agents.PackageAgent;


public final class Main {

	//TODO
	private static final int NUM_DEPOTS = 1;
	private static final int NUM_AGVS = 10;
	private static final double VEHICLE_LENGTH = 2d;
	private static final int NUM_PARCELS = 20;

	// time in ms
	//TODO
	private static final long SERVICE_DURATION = 60000;
	private static final int TAXI_CAPACITY = 10;
	private static final int DEPOT_CAPACITY = 100;

	//TODO
	private static final int SPEED_UP = 4;
	private static final int MAX_CAPACITY = 3;
	//TODO
	private static final double NEW_CUSTOMER_PROB = .000;

	//TODO
	private static final String MAP_FILE = "/data/maps/leuven-simple.dot";
	private static final Map<String, Graph<MultiAttributeData>> GRAPH_CACHE =
			newHashMap();

	//TODO
	private static final long TEST_STOP_TIME = 20 * 60 * 1000;
	private static final int TEST_SPEED_UP = 64;

	private Main() {}

	/*
	 * 
	 */
	public static void main(@Nullable String[] args) {
		//TODO
		final long endTime = args != null && args.length >= 1 ? Long
				.parseLong(args[0]) : Long.MAX_VALUE;

		final String graphFile = args != null && args.length >= 2 ? args[1]
				: MAP_FILE;
		run(false, endTime, graphFile, null /* new Display() */, null, null);
	}

	/*
	 * 
	 */
	public static void run(boolean testing) {
		//TODO
		run(testing, Long.MAX_VALUE, MAP_FILE, null, null, null);
	}

	/**
	 * Starts the example.
	 * @param testing Indicates whether the method should run in testing mode.
	 * @param endTime The time at which simulation should stop.
	 * @param graphFile The graph that should be loaded.
	 * @param display The display that should be used to show the ui on.
	 * @param m The monitor that should be used to show the ui on.
	 * @param list A listener that will receive callbacks from the ui.
	 * @return The simulator instance.
	 */
	public static Simulator run(boolean testing, final long endTime,
			String graphFile,
			@Nullable Display display, @Nullable Monitor m, @Nullable Listener list) {

		final View.Builder view = createGui(testing, display, m, list);

		final Simulator simulator = Simulator.builder()
				.addModel(RoadModelBuilders.dynamicGraph(GraphCreator.createSimpleGraph())
				          				   .withCollisionAvoidance()
				          				   .withDistanceUnit(SI.METER)
				          				   .withVehicleLength(VEHICLE_LENGTH))
				.addModel(DefaultPDPModel.builder())
				.addModel(CommModel.builder())
				.addModel(view)
				.build();
		
		final RandomGenerator rng = simulator.getRandomGenerator();

		final RoadModel roadModel = simulator.getModelProvider().getModel(
				RoadModel.class);
		
		final PDPModel pdpModel = simulator.getModelProvider().getModel(
				PDPModel.class);
		
		// add depots, taxis and parcels to simulator
		for (int i = 0; i < NUM_DEPOTS; i++) {
			simulator.register(new TaxiBase(roadModel.getRandomPosition(rng),
					DEPOT_CAPACITY));
		}
		for (int i = 0; i < NUM_AGVS; i++) {
			simulator.register(new AgvAgent(roadModel.getRandomPosition(rng),
					TAXI_CAPACITY, rng, roadModel));
		}
		for (int i = 0; i < NUM_PARCELS; i++) {
			simulator.register(new PackageAgent(
					Parcel.builder(roadModel.getRandomPosition(rng),
							roadModel.getRandomPosition(rng))
					.serviceDuration(SERVICE_DURATION)
					.neededCapacity(1 + rng.nextInt(MAX_CAPACITY))
					.buildDTO(),
					rng));
		}

		simulator.addTickListener(new TickListener() {
			@Override
			public void tick(TimeLapse time) {
				if (time.getStartTime() > endTime) {
					simulator.stop();
				} else if (rng.nextDouble() < NEW_CUSTOMER_PROB) {
					simulator.register(new PackageAgent(
							Parcel
							.builder(roadModel.getRandomPosition(rng),
									roadModel.getRandomPosition(rng))
							.serviceDuration(SERVICE_DURATION)
							.neededCapacity(1 + rng.nextInt(MAX_CAPACITY))
							.buildDTO(),
							rng));
				} 
				
				for (Parcel parcel : pdpModel.getParcels(ParcelState.DELIVERED)) {
					simulator.unregister(parcel);
				}
				
			}

			@Override
			public void afterTick(TimeLapse timeLapse) {}
		});
		simulator.start();

		return simulator;
	}

	static View.Builder createGui(
			boolean testing,
			@Nullable Display display,
			@Nullable Monitor m,
			@Nullable Listener list) {

		View.Builder view = View.builder()
								.with(WarehouseRenderer.builder()
								.withMargin(VEHICLE_LENGTH))
								.with(AGVRenderer.builder()
								.withDifferentColorsForVehicles())
								.with(CommRenderer.builder()
										.withReliabilityColors()
										.withToString()
										.withMessageCount());

		
		if (testing) {
			view = view.withAutoClose()
						.withAutoPlay()
						.withSimulatorEndTime(TEST_STOP_TIME)
						.withSpeedUp(TEST_SPEED_UP);
		} else if (m != null && list != null && display != null) {
			view = view.withMonitor(m)
						.withSpeedUp(SPEED_UP)
						.withResolution(m.getClientArea().width, m.getClientArea().height)
						.withDisplay(display)
						.withCallback(list)
						.withAsync()
						.withAutoPlay()
						.withAutoClose();
		}
		return view;
	}

	// currently has no function
	static class TaxiBase extends Depot {
		TaxiBase(Point position, double capacity) {
			super(position);
			setCapacity(capacity);
		}

		@Override
		public void initRoadPDP(RoadModel pRoadModel, PDPModel pPdpModel) {}
	}

  static class GraphCreator {
	    static final int LEFT_CENTER_U_ROW = 4;
	    static final int LEFT_CENTER_L_ROW = 5;
	    static final int LEFT_COL = 4;
	    static final int RIGHT_CENTER_U_ROW = 2;
	    static final int RIGHT_CENTER_L_ROW = 4;
	    static final int RIGHT_COL = 0;

	    GraphCreator() {}

	    static ImmutableTable<Integer, Integer, Point> createMatrix(int cols,
	        int rows, Point offset) {
	      final ImmutableTable.Builder<Integer, Integer, Point> builder =
	        ImmutableTable.builder();
	      for (int c = 0; c < cols; c++) {
	        for (int r = 0; r < rows; r++) {
	          builder.put(r, c, new Point(
	            offset.x + c * VEHICLE_LENGTH * 2,
	            offset.y + r * VEHICLE_LENGTH * 2));
	        }
	      }
	      return builder.build();
	    }

	    static ListenableGraph<LengthData> createSimpleGraph() {
	      final Graph<LengthData> g = new TableGraph<>();

	      final Table<Integer, Integer, Point> matrix = createMatrix(8, 6,
	        new Point(0, 0));

	      for (int i = 0; i < matrix.columnMap().size(); i++) {

	        final Iterable<Point> path;
	        if (i % 2 == 0) {
	          path = Lists.reverse(newArrayList(matrix.column(i).values()));
	        } else {
	          path = matrix.column(i).values();
	        }
	        Graphs.addPath(g, path);
	      }

	      Graphs.addPath(g, matrix.row(0).values());
	      Graphs.addPath(g, Lists.reverse(newArrayList(matrix.row(
	        matrix.rowKeySet().size() - 1).values())));

	      return new ListenableGraph<>(g);
	    }

	    static ListenableGraph<LengthData> createGraph() {
	      final Graph<LengthData> g = new TableGraph<>();

	      final Table<Integer, Integer, Point> leftMatrix = createMatrix(5, 10,
	        new Point(0, 0));
	      for (final Map<Integer, Point> column : leftMatrix.columnMap().values()) {
	        Graphs.addBiPath(g, column.values());
	      }
	      Graphs.addBiPath(g, leftMatrix.row(LEFT_CENTER_U_ROW).values());
	      Graphs.addBiPath(g, leftMatrix.row(LEFT_CENTER_L_ROW).values());

	      final Table<Integer, Integer, Point> rightMatrix = createMatrix(10, 7,
	        new Point(30, 6));
	      for (final Map<Integer, Point> row : rightMatrix.rowMap().values()) {
	        Graphs.addBiPath(g, row.values());
	      }
	      Graphs.addBiPath(g, rightMatrix.column(0).values());
	      Graphs.addBiPath(g, rightMatrix.column(rightMatrix.columnKeySet().size()
	        - 1).values());

	      Graphs.addPath(g,
	        rightMatrix.get(RIGHT_CENTER_U_ROW, RIGHT_COL),
	        leftMatrix.get(LEFT_CENTER_U_ROW, LEFT_COL));
	      Graphs.addPath(g,
	        leftMatrix.get(LEFT_CENTER_L_ROW, LEFT_COL),
	        rightMatrix.get(RIGHT_CENTER_L_ROW, RIGHT_COL));

	      return new ListenableGraph<>(g);
	    }
	  }
}
