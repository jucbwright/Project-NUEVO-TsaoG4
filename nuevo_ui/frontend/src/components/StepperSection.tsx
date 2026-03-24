import { useState, useCallback } from 'react';
import { useOptimisticEnable } from '../hooks/useOptimisticEnable';
import { ChevronDown, Home } from 'lucide-react';
import { Switch } from './ui/switch';
import { Input } from './ui/input';
import { useRobotStore } from '../store/robotStore';
import { wsSend } from '../lib/wsSend';
import { Modal } from './common/Modal';
import { StepperPlot } from './stepper/StepperPlot';
import { RecordCSV } from './common/RecordCSV';

interface StepperSectionProps {
  stepperId: number;
}

const MOTION_STATES = ['Idle', 'Accel', 'Cruise', 'Decel', 'Homing', 'Fault'];

export function StepperSection({ stepperId }: StepperSectionProps) {
  const motorState          = useRobotStore((s) => s.steppers[stepperId - 1]);
  const setStepperRecording = useRobotStore((s) => s.setStepperRecording);
  const status              = motorState?.status ?? null;

  const [showModal, setShowModal] = useState(false);
  const [maxVelocity, setMaxVelocity] = useState('1000');
  const [maxAcceleration, setMaxAcceleration] = useState('500');
  const [targetPosition, setTargetPosition] = useState('0');

  const isEnabled = (status?.enabled ?? 0) !== 0;
  const { switchChecked: enableSwitchChecked, dotEnabled, setOptimistic: setEnableOptimistic } =
    useOptimisticEnable(isEnabled);
  const currentPosition = status?.commandedCount ?? 0;

  const handleEnable = (checked: boolean) => {
    setEnableOptimistic(checked);
    wsSend('step_enable', { stepperNumber: stepperId, enable: checked ? 1 : 0 });
  };

  const handleApplyParams = () => {
    wsSend('step_config_set', {
      stepperNumber: stepperId,
      maxVelocity: parseFloat(maxVelocity),
      acceleration: parseFloat(maxAcceleration),
    });
  };

  const handleMove = () => {
    wsSend('step_move', {
      stepperNumber: stepperId,
      moveType: 0,
      target: parseInt(targetPosition),
    });
  };

  const handleHome = () => {
    wsSend('step_home', { stepperNumber: stepperId });
  };

  // ── Recording ────────────────────────────────────────────────────────────
  const handleRecordingChange = useCallback((isRecording: boolean) => {
    setStepperRecording(stepperId - 1, isRecording);
  }, [stepperId, setStepperRecording]);

  const getData = useCallback((): number[][] => {
    const ms = useRobotStore.getState().steppers[stepperId - 1];
    const { timeHistory, positionHistory, velocityHistory, recordingStartTs } = ms;

    let startIdx = 0;
    if (recordingStartTs !== null) {
      const found = timeHistory.findIndex((t) => t >= recordingStartTs);
      startIdx = found >= 0 ? found : 0;
    }

    const n = timeHistory.length - startIdx;
    if (n === 0) return [[], [], []];

    const t0 = timeHistory[startIdx];
    return [
      timeHistory.slice(startIdx).map((t) => (t - t0) / 1000),
      positionHistory.slice(startIdx),
      velocityHistory.slice(startIdx),
    ];
  }, [stepperId]);

  // ── Modal header: enable switch ──────────────────────────────────────────
  const enableAction = (
    <div className="flex items-center gap-2">
      <div className={`size-2 rounded-full transition-all ${dotEnabled ? 'bg-emerald-400 shadow-lg shadow-emerald-400/50' : 'bg-white/30'}`} />
      <span className="text-xs text-white/60">{dotEnabled ? 'Enabled' : 'Disabled'}</span>
      <Switch checked={enableSwitchChecked} onCheckedChange={handleEnable} />
    </div>
  );

  return (
    <>
      {/* ── Card ─────────────────────────────────────────────────────────── */}
      <div
        onClick={() => setShowModal(true)}
        className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full cursor-pointer hover:bg-white/15 transition-all group"
      >
        <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent" />
        <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50" />

        <div className="relative">
          <div className="flex items-center justify-between mb-3">
            <h3 className="text-sm font-semibold text-white whitespace-nowrap">Stepper {stepperId}</h3>
            <ChevronDown className="size-4 text-white/70 group-hover:text-white transition-colors" />
          </div>

          <div className="flex items-center justify-between mb-2">
            <div className="flex items-center gap-2">
              <div className={`size-2 rounded-full transition-all ${dotEnabled ? 'bg-emerald-400 shadow-lg shadow-emerald-400/50' : 'bg-white/30'}`} />
              <span className="text-xs text-white/60">{dotEnabled ? 'Enabled' : 'Disabled'}</span>
            </div>
            <Switch
              checked={enableSwitchChecked}
              onCheckedChange={handleEnable}
              onClick={(e) => e.stopPropagation()}
            />
          </div>

          <div className="mt-3 pt-3 border-t border-white/10">
            <div className="text-sm text-white/50 text-left">Position: {currentPosition}</div>
          </div>
        </div>
      </div>

      {/* ── Modal ────────────────────────────────────────────────────────── */}
      <Modal
        open={showModal}
        onClose={() => setShowModal(false)}
        title={`Stepper ${stepperId}`}
        subtitle={status
          ? `Pos: ${status.commandedCount} steps · Vel: ${status.currentSpeed} steps/s · ${MOTION_STATES[status.motionState] ?? 'Unknown'}`
          : undefined
        }
        headerActions={enableAction}
        maxWidth="max-w-2xl"
      >
        <div className="space-y-5">

          {/* ── Live status ──────────────────────────────────────────────── */}
          <div className="rounded-2xl p-4 backdrop-blur-xl bg-white/5 border border-white/10 space-y-2">
            <div className="flex justify-between">
              <span className="text-xs text-white/60">Position</span>
              <span className="text-xs font-mono text-white">{status?.commandedCount ?? '--'} steps</span>
            </div>
            <div className="flex justify-between">
              <span className="text-xs text-white/60">Target</span>
              <span className="text-xs font-mono text-white">{status?.targetCount ?? '--'} steps</span>
            </div>
            <div className="flex justify-between">
              <span className="text-xs text-white/60">Velocity</span>
              <span className="text-xs font-mono text-white">{status?.currentSpeed ?? '--'} steps/s</span>
            </div>
            <div className="flex justify-between">
              <span className="text-xs text-white/60">Motion</span>
              <span className="text-xs font-mono text-white">
                {status ? (MOTION_STATES[status.motionState] ?? 'Unknown') : '--'}
              </span>
            </div>
            {(status?.limitHit ?? 0) !== 0 && (
              <div className="flex justify-between">
                <span className="text-xs text-amber-400">Limit Hit</span>
                <span className="text-xs font-mono text-amber-400">
                  {(status!.limitHit & 1) ? 'Min ' : ''}{(status!.limitHit & 2) ? 'Max' : ''}
                </span>
              </div>
            )}
          </div>

          {/* ── Motion params ─────────────────────────────────────────────── */}
          <div className="flex items-end gap-3">
            <div className="flex-1">
              <label className="text-xs text-white/70 mb-1.5 block">Max Velocity (steps/s)</label>
              <Input
                type="number"
                value={maxVelocity}
                onChange={(e) => setMaxVelocity(e.target.value)}
                className="backdrop-blur-xl bg-white/10 border-white/20 text-white"
              />
            </div>
            <div className="flex-1">
              <label className="text-xs text-white/70 mb-1.5 block">Max Acceleration (steps/s²)</label>
              <Input
                type="number"
                value={maxAcceleration}
                onChange={(e) => setMaxAcceleration(e.target.value)}
                className="backdrop-blur-xl bg-white/10 border-white/20 text-white"
              />
            </div>
            <button
              onClick={handleApplyParams}
              className="px-4 py-2 rounded-xl backdrop-blur-xl bg-white/10 border border-white/20 hover:bg-white/20 transition-all text-sm font-semibold text-white flex-shrink-0"
            >
              Apply
            </button>
          </div>

          {/* ── Move ─────────────────────────────────────────────────────── */}
          <div>
            <label className="text-xs text-white/70 mb-1.5 block">Target Position (steps)</label>
            <div className="flex gap-3">
              <Input
                type="number"
                value={targetPosition}
                onChange={(e) => setTargetPosition(e.target.value)}
                className="flex-1 backdrop-blur-xl bg-white/10 border-white/20 text-white"
              />
              <button
                onClick={handleMove}
                className="px-4 py-2 rounded-xl backdrop-blur-xl bg-cyan-500/30 border border-cyan-400/50 hover:bg-cyan-500/40 transition-all text-sm font-semibold text-white"
              >
                Move
              </button>
              <button
                onClick={handleHome}
                className="flex items-center gap-2 px-4 py-2 rounded-xl backdrop-blur-xl bg-amber-500/20 border border-amber-400/40 hover:bg-amber-500/30 transition-all text-sm font-semibold text-white"
              >
                <Home className="size-4" />
                Home
              </button>
            </div>
          </div>

          {/* ── Recording ─────────────────────────────────────────────────── */}
          <RecordCSV
            headers={['Time (s)', 'Position (steps)', 'Velocity (steps/s)']}
            getData={getData}
            filename={`stepper_${stepperId}`}
            onRecordingChange={handleRecordingChange}
          />

          {/* ── Live plot ─────────────────────────────────────────────────── */}
          <div className="rounded-2xl p-4 bg-white/5 border border-white/10">
            <h3 className="text-sm font-semibold text-white mb-3">Real-time Data</h3>
            <StepperPlot
              timeHistory={motorState?.timeHistory ?? []}
              positionHistory={motorState?.positionHistory ?? []}
              velocityHistory={motorState?.velocityHistory ?? []}
              recordingStartTs={motorState?.recordingStartTs ?? null}
            />
          </div>

        </div>
      </Modal>
    </>
  );
}
