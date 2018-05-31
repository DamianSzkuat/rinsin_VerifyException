package project.agents;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import javax.measure.Measure;
import javax.measure.quantity.Duration;
import javax.measure.quantity.Length;
import javax.measure.quantity.Velocity;
import javax.measure.unit.Unit;

import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.pdp.PDPModel;
import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.core.model.pdp.VehicleDTO;
import com.github.rinde.rinsim.core.model.road.CollisionGraphRoadModelImpl;
import com.github.rinde.rinsim.core.model.road.RoadModel;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.geom.GeomHeuristics;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.base.Optional;


public class AgvAgent extends Vehicle implements CommUser{
	
	// Variables associated with the movement of the AGV
	private static final double SPEED = 2d;
	RoadModel roadModel;
	
	// Variables associated with the communication of the AGV
	Optional<CommDevice> commDevice;
	private final double reliability;
	static final double MIN_RELIABILITY = 0.9;
	static final double MAX_RELIABILITY = 1.0;
	private final double range;
	static final double MIN_RANGE = 5;
	static final double MAX_RANGE = 5.5;
	static final long LONELINESS_THRESHOLD = 10 * 1000;
	long lastReceiveTime;
	private final Map<CommUser, Integer> reachableAgents;
	static final Integer MAX_AVAILABILITY_SCORE = 20;
	//private Set<AntAgent> antAgents;
	//private Set<AntAgent> agentsToBeDeployed;
	//Map<AntAgent, CommUser> antsToMigrate = new HashMap<>();
	
	// Variables associated with the planning of the AGV
	//private AgvScheduler scheduler;
	//private float bateryCharge;
	private Optional<PackageAgent> packageAgent;
	private long currentTime;

	// Various
	private final RandomGenerator rng;
	private Optional<Point> randomDestination;
	private Queue<Point> randomPath;
	public Unit<Duration> timeUnit;

	public AgvAgent(Point startPosition, int capacity, RandomGenerator r, RoadModel roadModel) {
		super(VehicleDTO.builder()
			      		.capacity(capacity)
			      		.startPosition(startPosition)
			      		.speed(SPEED)
			      		.build());
		packageAgent = Optional.absent();
		commDevice = Optional.absent();
		randomDestination = Optional.absent();
		
		//scheduler = new AgvScheduler(this);
		
		// TODO Auto-generated constructor stub
		rng = r;
		range = MIN_RANGE + rng.nextDouble() * (MAX_RANGE - MIN_RANGE); 
		reliability = MIN_RELIABILITY + rng.nextDouble() * (MAX_RELIABILITY - MIN_RELIABILITY);
		reachableAgents = new HashMap<>();
		this.roadModel = roadModel;
		randomDestination = Optional.absent();
		randomPath = new LinkedList<>();
	}

//	public Map<CommUser, Integer> getReachableAgents() {
//		return this.reachableAgents;
//	}
//	
//	private void addToReachableAgents(CommUser commUser) {
//		if( commUser instanceof AgvAgent ) {
//			getReachableAgents().put(commUser, PackageAgent.MAX_AVAILABILITY_SCORE);
//		}
//	}
	
	long getCurrentTime() {
		return this.currentTime;
	}
	
	/********************************************************************/
	// Transport Negotiations                                           //
	/********************************************************************/
	
	long getTimeToReachGivenPoint(Point point) {

		Measure<Double, Velocity> maxSpeed = Measure.valueOf(getSpeed(), getRoadModel().getSpeedUnit());
		
		List<Point> path = new LinkedList<>(getRoadModel().getPathTo(this, point, this.timeUnit, maxSpeed,
																	 GeomHeuristics.euclidean()).getPath());
		
		Measure<Double,Length> distance = getRoadModel().getDistanceOfPath(path);	
		long time = (long) (distance.getValue()/AgvAgent.SPEED);
		return time;
	}
	
	long getTimeToMoveBetweenTwoPoints(Point start, Point end) {
		Measure<Double,Length> distance = getRoadModel().getDistanceOfPath(getRoadModel().getShortestPathTo(start, end));
		long time = (long) (distance.getValue()/AgvAgent.SPEED);
		return time;
	}
	
	/********************************************************************/
	// Vehicle                                                          //
	/********************************************************************/
	
	@Override
	public void afterTick(TimeLapse timeLapse) {
	}
	
	@Override
	protected void tickImpl(TimeLapse time) {
		// TODO Auto-generated method stub
		currentTime = time.getStartTime();
		timeUnit = time.getTimeUnit();
						
		// Planning //
		planningTick();
		
		// Movement // 
		movementTick(time);

	}
	
	private void planningTick() {
		
	}
	
	private void movementTick(TimeLapse time) {
		roadModel = getRoadModel();
		final PDPModel pdpModel = getPDPModel();
		
		// Is there time left to comsume? 
		if (!time.hasTimeLeft()) {
			// Stop if not
			return;
		}
		
		// Is there a parcel on the AGV?
		if (!packageAgent.isPresent()) {
			// If not look for one 
			// TODO 
			
			CollisionGraphRoadModelImpl model = (CollisionGraphRoadModelImpl) getRoadModel();
		   
			if (!randomDestination.isPresent()) {
			      nextRandomDestination(model);
			    }

			getRoadModel().moveTo(this, randomDestination.get(), time);


		    if (model.getPosition(this).equals(randomDestination.get())) {
		      nextRandomDestination(model);
		    }
		}
		
		// Is there a parcel on the AGV?
		if (packageAgent.isPresent()) {
			// If yes, check if it is in the container.
			final boolean inCargo = pdpModel.containerContains(this, packageAgent.get());
			
			// Sanity check: if it is not in our cargo AND it is also not on the
		    // RoadModel, we cannot go to curr anymore.
			if (!inCargo && !roadModel.containsObject(packageAgent.get())) {
				packageAgent = Optional.absent();
			}
			else if (inCargo) {
				// If the parcel is in cargo, move to its destination
				roadModel.moveTo(this, packageAgent.get().getDeliveryLocation(), time);
				
				// If we arrived at the parcel delivery location
				if (roadModel.getPosition(this).equals(packageAgent.get().getDeliveryLocation())) {
					// Deliver the parcel
					pdpModel.deliver(this, packageAgent.get(), time);
					packageAgent.get().notifyDropOff(roadModel.getPosition(this));
				}
			} else {
				// The parcel is still available, go to pick it up
				roadModel.moveTo(this, packageAgent.get(), time);
				// If you are at the parcel position
				if (roadModel.equalPosition(this, packageAgent.get())) {
					// Pick up parcel
					packageAgent.get().notifyOfPickUp(this);
					pdpModel.pickup(this, packageAgent.get(), time);
				}
			}
		}
	}
	
	void nextRandomDestination(CollisionGraphRoadModelImpl model) {
		randomDestination = Optional.of(model.getRandomPosition(rng));
		
		Point end = randomDestination.get();
		
		Point start = model.getPosition(this);
		if (model.getConnection(this).isPresent()) {
			  start = model.getConnection(this).get().to();
		}
		
	    randomPath = new LinkedList<>(model.getShortestPathTo(start, end));
	  }
	
	
	/********************************************************************/
	// CommUser                                                         //
	/********************************************************************/
	
	@Override
	public Optional<Point> getPosition() {
	    if (roadModel.containsObject(this)) {
	    	return Optional.of(roadModel.getPosition(this));
	    }
	    return Optional.absent();
	}

	@Override
	public void setCommDevice(CommDeviceBuilder builder) {
	    if (range >= 0) {
	    	builder.setMaxRange(range);
	    }
	    commDevice = Optional.of(builder
	    				.setReliability(reliability)
	    				.build());
	}

}











