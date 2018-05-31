package project.agents;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.apache.commons.math3.random.RandomGenerator;

import com.github.rinde.rinsim.core.model.pdp.PDPModel.ParcelState;
import com.github.rinde.rinsim.core.model.time.TickListener;
import com.github.rinde.rinsim.core.model.time.TimeLapse;
import com.github.rinde.rinsim.core.model.comm.CommDevice;
import com.github.rinde.rinsim.core.model.comm.CommDeviceBuilder;
import com.github.rinde.rinsim.core.model.comm.CommUser;
import com.github.rinde.rinsim.core.model.pdp.Parcel;
import com.github.rinde.rinsim.core.model.pdp.ParcelDTO;
import com.github.rinde.rinsim.core.model.pdp.Vehicle;
import com.github.rinde.rinsim.geom.Point;
import com.google.common.base.Optional;

public class PackageAgent extends Parcel implements CommUser, TickListener{
	
	private Optional<Vehicle> currentTransport;
	private Optional<Point> dropOffPosition;
	
	//private boolean sendNewPotentialReservationRequest;
	
	Set<ParcelState> staticStates = new HashSet<>();
	Set<ParcelState> movingStates = new HashSet<>();
		
	Optional<CommDevice> commDevice;
	private final double reliability;
	static final double MIN_RELIABILITY = 0.9;
	static final double MAX_RELIABILITY = 1.0;
	private final double range;
	private final RandomGenerator rng;
	static final double MIN_RANGE = 5.0;
	static final double MAX_RANGE = 5.5;
	static final int MAX_RESENDS = 3;
	long lastReceiveTime;
	long lastReservationRequestSendTime;
	static final Integer MAX_AVAILABILITY_SCORE = 20;
	private final Map<CommUser, Integer> reachableAgents;
	static final long LONELINESS_THRESHOLD = 100 * 1000;
	
	public PackageAgent(ParcelDTO parcelDto, RandomGenerator r) {
		super(parcelDto);
		
		currentTransport = Optional.absent();
		dropOffPosition = Optional.absent();
		
		//sendNewPotentialReservationRequest = true;
		
		// TODO Auto-generated constructor stub
		initParcelStateSets();
		
		rng = r;
		range = MIN_RANGE + rng.nextDouble() * (MAX_RANGE - MIN_RANGE);
		reliability = MIN_RELIABILITY + rng.nextDouble() * (MAX_RELIABILITY - MIN_RELIABILITY);
		
		reachableAgents = new HashMap<>();
	}
	
	public void notifyOfPickUp(Vehicle pickUpVehicle) {
		//dropOffPosition = Optional.absent();
		setCurrentTransport(pickUpVehicle);
	}
	
	public void notifyDropOff(Point dropLocation) {
		//currentTransport = Optional.absent();
		setDropOffPosition(dropLocation);
	}
	
	public void notifyDelivery(Point deliveryLocation) {
		//currentTransport = Optional.absent();
		setDropOffPosition(deliveryLocation);
	}
		
	private void initParcelStateSets() {
		// Adding static states a.k.a parcel is not moving
		staticStates.add(ParcelState.ANNOUNCED);
		staticStates.add(ParcelState.AVAILABLE);
		staticStates.add(ParcelState.PICKING_UP);
		staticStates.add(ParcelState.DELIVERED);
		movingStates.add(ParcelState.DELIVERING);
		
		// Adding moving states
		movingStates.add(ParcelState.IN_CARGO);
	}
	
	Optional<Vehicle> getCurrentTransport() {
		return this.currentTransport;
	}
	
	void setCurrentTransport(Vehicle pickUpVehicle) {
		currentTransport = Optional.of(pickUpVehicle);
	}
	
	Optional<Point> getDropOffPosition() {
		return this.getDropOffPosition();
	}
	
	void setDropOffPosition(Point dropLocation) {
		dropOffPosition = Optional.of(dropLocation);
	}
	
	public Map<CommUser, Integer> getReachableAgents() {
		return this.reachableAgents;
	}
	
	private void addToReachableAgents(CommUser commUser) {
		if( commUser instanceof AgvAgent ) {
			getReachableAgents().put(commUser, PackageAgent.MAX_AVAILABILITY_SCORE);
		}
	}
	
	/********************************************************************/
	// TickListener                                                     //
	/********************************************************************/
	
	@Override
	public void tick(TimeLapse time) {
		// TODO Auto-generated method stub
		// Communication //
		
		//intentionAntMigration();
	}
	
	@Override
	public void afterTick(TimeLapse time) {

	}
	
	/********************************************************************/
	// CommUser                                                         //
	/********************************************************************/
	
	@Override
	public Optional<Point> getPosition() {
		ParcelState parcelState = this.getPDPModel().getParcelState(this);
		
		if(staticStates.contains(parcelState)) {
			if (dropOffPosition.isPresent()) {
				return dropOffPosition;
			} else {
				return Optional.of(this.getPickupLocation());
			}
		}
		else if(movingStates.contains(parcelState)) {
			return Optional.of(this.getRoadModel().getPosition(currentTransport.get()));
		}
		else {
			throw new IllegalArgumentException("The parcel state is invalid.");
		}
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
