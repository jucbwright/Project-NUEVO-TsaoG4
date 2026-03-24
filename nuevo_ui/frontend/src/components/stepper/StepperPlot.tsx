/**
 * StepperPlot — real-time uPlot chart showing position and velocity.
 * X-axis: "seconds ago", newest at left (0).
 * Recording overlay: red dashed bar + fill while recording.
 */
import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import './StepperPlot.css'

interface StepperPlotProps {
  timeHistory: number[]
  positionHistory: number[]
  velocityHistory: number[]
  recordingStartTs: number | null
}

export function StepperPlot({
  timeHistory,
  positionHistory,
  velocityHistory,
  recordingStartTs,
}: StepperPlotProps) {
  const chartRef = useRef<HTMLDivElement>(null)
  const plotRef  = useRef<uPlot | null>(null)
  const animRef  = useRef<number | null>(null)
  const seriesVisRef = useRef<boolean[]>([true, true])

  const timeRef     = useRef(timeHistory)
  const posRef      = useRef(positionHistory)
  const velRef      = useRef(velocityHistory)
  const recStartRef = useRef(recordingStartTs)
  const xMaxRef     = useRef(20)

  useEffect(() => { timeRef.current     = timeHistory     }, [timeHistory])
  useEffect(() => { posRef.current      = positionHistory }, [positionHistory])
  useEffect(() => { velRef.current      = velocityHistory }, [velocityHistory])
  useEffect(() => { recStartRef.current = recordingStartTs }, [recordingStartTs])

  // Create plot once
  useEffect(() => {
    if (!chartRef.current) return

    const opts: uPlot.Options = {
      width:  chartRef.current.clientWidth,
      height: 180,
      series: [
        { label: 'Seconds Ago' },
        { label: 'Position (steps)', stroke: '#4ade80', width: 2, scale: 'pos', show: true },
        { label: 'Velocity (steps/s)', stroke: '#3b82f6', width: 2, scale: 'vel', show: true },
      ],
      scales: {
        x:   { time: false, range: (_u, _dmin, _dmax) => [0, xMaxRef.current] },
        pos: { auto: true },
        vel: { auto: true },
      },
      axes: [
        { label: 'Seconds Ago',      stroke: '#9ca3af', grid: { stroke: '#333', width: 1 } },
        { label: 'Position (steps)', stroke: '#4ade80', grid: { stroke: '#333', width: 1 }, scale: 'pos', side: 3 },
        { label: 'Velocity (steps/s)', stroke: '#3b82f6', grid: { show: false },            scale: 'vel', side: 1 },
      ],
      legend: { show: true, live: false },
      cursor: { drag: { x: false, y: false } },
      hooks: {
        draw: [
          (u: uPlot) => {
            const startTs = recStartRef.current
            if (!startTs) return
            const th = timeRef.current
            if (th.length === 0) return
            const elapsed = (th[th.length - 1] - startTs) / 1000
            if (elapsed <= 0) return
            const x0   = Math.round(u.valToPos(0,       'x', true))
            const barX = Math.round(u.valToPos(elapsed, 'x', true))
            const top    = u.bbox.top
            const height = u.bbox.height
            const ctx = u.ctx
            ctx.save()
            ctx.fillStyle = 'rgba(239, 68, 68, 0.08)'
            ctx.fillRect(x0, top, barX - x0, height)
            ctx.strokeStyle = 'rgba(239, 68, 68, 0.75)'
            ctx.lineWidth = 1.5
            ctx.setLineDash([5, 4])
            ctx.beginPath()
            ctx.moveTo(barX, top)
            ctx.lineTo(barX, top + height)
            ctx.stroke()
            ctx.restore()
          },
        ],
      },
    }

    const emptyData: uPlot.AlignedData = [
      new Float64Array(0),
      new Float64Array(0),
      new Float64Array(0),
    ]

    const plot = new uPlot(opts, emptyData, chartRef.current)
    plotRef.current = plot

    const legend = plot.root.querySelector('.u-legend')
    if (legend) {
      legend.addEventListener('click', (e) => {
        const p = plotRef.current
        if (!p) return
        const target = (e.target as HTMLElement).closest('.u-series') as HTMLElement | null
        if (!target) return
        const items = legend.querySelectorAll('.u-series')
        const idx = Array.from(items).indexOf(target)
        if (idx < 0) return
        seriesVisRef.current[idx] = !seriesVisRef.current[idx]
        p.setSeries(idx + 1, { show: seriesVisRef.current[idx] })
      })
    }

    return () => {
      plot.destroy()
      plotRef.current = null
    }
  }, [])

  // Persistent animation loop
  useEffect(() => {
    const tick = () => {
      const plot = plotRef.current
      const th = timeRef.current
      if (plot && th.length > 0) {
        const n      = th.length
        const newest = th[n - 1]
        const startTs = recStartRef.current
        xMaxRef.current = startTs !== null
          ? Math.max(20, (newest - startTs) / 1000)
          : 20

        const xArr   = new Float64Array(n)
        const posArr = new Float64Array(n)
        const velArr = new Float64Array(n)
        const pos = posRef.current
        const vel = velRef.current
        for (let i = 0; i < n; i++) {
          const j  = n - 1 - i
          xArr[i]  = (newest - th[j]) / 1000
          posArr[i] = pos[j]
          velArr[i] = vel[j]
        }
        plot.setData([xArr, posArr, velArr])
      }
      animRef.current = requestAnimationFrame(tick)
    }
    animRef.current = requestAnimationFrame(tick)
    return () => { if (animRef.current) cancelAnimationFrame(animRef.current) }
  }, [])

  // Resize handler
  useEffect(() => {
    const onResize = () => {
      if (plotRef.current && chartRef.current) {
        plotRef.current.setSize({ width: chartRef.current.clientWidth, height: 180 })
      }
    }
    window.addEventListener('resize', onResize)
    return () => window.removeEventListener('resize', onResize)
  }, [])

  return (
    <div ref={chartRef} className="stepper-plot">
      {timeHistory.length === 0 && (
        <div className="stepper-plot-empty">Waiting for data…</div>
      )}
    </div>
  )
}
