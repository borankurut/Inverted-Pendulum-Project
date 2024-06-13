#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtkCallbackCommand.h>
#include <vtkProperty.h>
#include "vector.h"
#include <iostream>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>

Vec3* SharedPosition_p;

Vec3 CartDimensions{7, 7, 7};   // all 7 for simplicity.
Vec3 RodDimensions{1, 39.2, 1}; // values measured.

float* SharedAngle_p;

// custom callback class because I can't find a function that updates the states of objects.
class TimerCallback : public vtkCommand
{
public:
    static TimerCallback* New()
    {
        return new TimerCallback;
    }

    virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData) override
    {
        if (eventId == vtkCommand::TimerEvent)
        {
            vtkRenderWindowInteractor* interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);

            SetSharedPosition();
			SetSharedAngle();
            MoveToDesiredPosition();
			MoveToDesiredAngle();

            _renderer->GetRenderWindow()->Render();
        }
    }

    void SetCartActor(vtkActor* actor)
    {
        _cartActor = actor;
    }

    void SetRodActor(vtkActor* actor)
    {
        _rodActor = actor;
    }

    void SetRenderer(vtkRenderer* ren)
    {
        _renderer = ren;
    }

    void SetRodOffset(const Vec3& rodOffset){
        _rodOffset = rodOffset;
    }

    void SetMinMaxPosition(double min, double max)
    {
        _minPos = min;
        _maxPos = max;
    }

    void SetDesiredPosition(const Vec3& desiredPos){
        Vec3 actorPos = double_array_to_vector(this->_cartActor->GetPosition());

        if(desiredPos.x() > _maxPos || desiredPos.x() < _minPos || desiredPos.y() != actorPos.y() || desiredPos.z() != actorPos.z()){
            std::cout << "can't set desiredPos: " << desiredPos << std::endl;
            return;
        }
        else
            this->_desiredPos = desiredPos;
    }

    void SetSharedPosition(){
        this->_desiredPos = *SharedPosition_p;
		std::cout << "Current position of the cart is at " << _desiredPos.x() << "cm." << std::endl << std::endl;
    }

    void MoveToDesiredPosition(){
        Vec3 actorPos = double_array_to_vector(this->_cartActor->GetPosition());
        if((_desiredPos - actorPos).length() < 0.01f)
            return;

        double* to_set = vector_to_double_array(_desiredPos);
        double* to_set_rod = vector_to_double_array(_desiredPos + _rodOffset);
        _cartActor->SetPosition(to_set);
        _rodActor->SetPosition(to_set_rod);
        free(to_set);
        free(to_set_rod);
    }

    void MoveToDesiredAngle(){
		double *orient = _rodActor->GetOrientation();
		_rodActor->SetOrientation(orient[0], orient[1], _desiredAngle);
	}

	void SetSharedAngle(){
		this->_desiredAngle = *SharedAngle_p;
		std::cout << "Current angle of the rod is at " << _desiredAngle << " degrees. " << std::endl << std::endl;
	}

private:
    vtkActor* _cartActor;
    vtkActor* _rodActor;
    vtkRenderer* _renderer;

	Vec3 _rodOffset = {0, 0, 0};

    double _minPos = -5.0;
    double _maxPos = 5.0;

    Vec3 _desiredPos = {0, 0, 0};
	double _desiredAngle = 0;

};

void TimerCallbackFunction(vtkObject* caller, unsigned long eventId, void* clientData, void* callData)
{
    TimerCallback* callback = static_cast<TimerCallback*>(clientData);
    callback->Execute(caller, eventId, callData);
}

int main()
{   
	// shared memory part
    key_t key = ftok("shmfile", 65);

    // shmget returns an identifier in shmid
    int shmid = shmget(key, sizeof(Vec3) + sizeof(float) , 0666 | IPC_CREAT);

    // shmat to attach to shared memory
    void *SharedMemory = (Vec3*)shmat(shmid, nullptr, 0);

    SharedPosition_p = static_cast<Vec3*>(SharedMemory);
    SharedAngle_p = reinterpret_cast<float*>(static_cast<char *>(SharedMemory) + sizeof(Vec3));  // Offset to get the next location in shared memory

    vtkNew<vtkNamedColors> colors;

	// basic structures of a vtk scene.
	
    // Create source
    vtkSmartPointer<vtkCubeSource> cartSource = vtkSmartPointer<vtkCubeSource>::New();
    cartSource->SetXLength(CartDimensions.x());
    cartSource->SetYLength(CartDimensions.y());
    cartSource->SetZLength(CartDimensions.z());
    vtkSmartPointer<vtkCubeSource> rodSource = vtkSmartPointer<vtkCubeSource>::New();
    rodSource->SetXLength(RodDimensions.x());
    rodSource->SetYLength(RodDimensions.y());
    rodSource->SetZLength(RodDimensions.z());

    // Create mapper
    vtkSmartPointer<vtkPolyDataMapper> mapperCart = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapperCart->SetInputConnection(cartSource->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> mapperRod = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapperRod->SetInputConnection(rodSource->GetOutputPort());

    // Create actor
    vtkSmartPointer<vtkActor> cartActor = vtkSmartPointer<vtkActor>::New();
    cartActor->SetMapper(mapperCart);
    cartActor->GetProperty()->SetColor(0.2, 0.2, 0.2);

    vtkSmartPointer<vtkActor> rodActor = vtkSmartPointer<vtkActor>::New();
    rodActor->SetMapper(mapperRod);
    rodActor->GetProperty()->SetColor(0.5, 0.5, 0.5);
	rodActor->SetOrigin(0.0, -RodDimensions.y()/2, 0.0);

    // Create renderer
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(181/256.0, 176/256.0, 173/256.0);

    // Add actor to the scene
    renderer->AddActor(cartActor);
    renderer->AddActor(rodActor);

    // Create render window
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(960, 1080);

    // Create a virtual camera
    vtkNew<vtkCamera> orientation;
    orientation->SetFocalPoint(0,0,0);
    orientation->SetPosition(0, 50, 120);
    renderer->SetActiveCamera(orientation);

    // Create render window interactor
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Initialize the interactor before adding the observer
    renderWindowInteractor->Initialize();

    // Create a timer callback for continuously moving the cube
    vtkSmartPointer<TimerCallback> callback = vtkSmartPointer<TimerCallback>::New();
    callback->SetCartActor(cartActor);
    callback->SetRodActor(rodActor);
    callback->SetRenderer(renderer);
    callback->SetRodOffset({0, RodDimensions.y()/2, RodDimensions.z()/2 - CartDimensions.z()/2 + CartDimensions.z()});
    callback->SetMinMaxPosition(-10.0, 10.0);

    // Create a timer to trigger the callback periodically
    vtkSmartPointer<vtkCallbackCommand> timerCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    timerCallback->SetCallback(TimerCallbackFunction);
    timerCallback->SetClientData(callback); // Pass the callback object as client data
    renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, timerCallback);
    renderWindowInteractor->AddObserver(vtkCommand::LeftButtonPressEvent, timerCallback);


    // Start the timer
    renderWindowInteractor->CreateRepeatingTimer(10); // 10 milliseconds

    // Start the interaction
    renderWindowInteractor->Start();


	// shared memory cleaning
    // detach from shared memory
    shmdt(SharedPosition_p);
    shmdt(SharedAngle_p);
 
    // destroy the shared memory
    shmctl(shmid, IPC_RMID, NULL);

    return 0;
}

